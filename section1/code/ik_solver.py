"""
ik_solver.py — Section 1 IK helper library.

Wraps MoveIt 2's standard `/compute_ik` service into a small Python class that:
  * builds a surface-aligned end-effector orientation,
  * blocks (with timeout) on the async service call without deadlocking
    a MultiThreadedExecutor,
  * returns a structured result (success / joint angles / rejection reason).

The reachability heatmap and the standalone IK service node both use this
class so all IK logic lives in one place.

Surface orientation convention (tool/TCP frame = link6):
  countertop  → tool Z-axis points down  (world -Z)  : RPY = [π, 0, yaw]
  mirror      → tool Z-axis points into the mirror face (world +Y) : RPY = [π/2, 0, yaw]

`tcp_yaw` rotates the tool about its own Z-axis (final wrist roll) so the
planner can try different approach angles when the default fails.

`tilt_deg` tilts the tool away from the surface normal by the given amount
(in degrees) about a chosen axis in the tool frame (default: tool +X). This
implements the "lean around the faucet" / "lean into reach" trick — staying
within the assignment's ±10° surface-normal tolerance.

Arm group / kinematic chain — defined in piper_with_gripper_moveit/config/piper.srdf
  group     : "arm"
  base_link : "base_link"
  tip_link  : "link6"
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np
from scipy.spatial.transform import Rotation

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState

ARM_GROUP = "arm"
TIP_LINK = "link6"
PIPER_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


@dataclass
class IKResult:
    success: bool
    reason: str = ""
    joint_angles: List[float] = field(default_factory=list)


def surface_quaternion(surface: str, tcp_yaw: float = 0.0,
                       tilt_rad: float = 0.0,
                       tilt_axis: str = "x") -> np.ndarray:
    """Return [x, y, z, w] for a tool aligned with the named surface.

    The tilt is applied in the *tool* frame (post-multiplied) so the tool
    still rolls about its (possibly tilted) Z-axis by tcp_yaw.
    Order: base_orientation · tilt(about tool tilt_axis) · yaw(about tool Z).
    """
    if surface == "mirror":
        # Rotate -π/2 about world X so tool Z (gripper approach axis) points
        # in world +Y — i.e. INTO the mirror face. The +π/2 form aimed the
        # gripper into the room instead of the glass, breaking all mirror IK.
        base = Rotation.from_euler("xyz", [-math.pi / 2, 0.0, 0.0])
    elif surface == "countertop":
        base = Rotation.from_euler("xyz", [math.pi, 0.0, 0.0])
    else:
        raise ValueError(f"surface must be 'countertop' or 'mirror', got {surface!r}")
    if tilt_rad:
        base = base * Rotation.from_euler(tilt_axis, tilt_rad)
    if tcp_yaw:
        base = base * Rotation.from_euler("z", tcp_yaw)
    return base.as_quat()


class IKSolver:
    """Thin convenience wrapper around MoveIt's /compute_ik service."""

    def __init__(self, node: Node, service_timeout: float = 10.0,
                 ik_timeout_sec: float = 0.3, call_timeout_sec: float = 2.0):
        self._node = node
        self._call_timeout = call_timeout_sec
        self._ik_timeout = ik_timeout_sec

        self._client = node.create_client(GetPositionIK, "/compute_ik")
        node.get_logger().info("IKSolver: waiting for /compute_ik …")
        if not self._client.wait_for_service(timeout_sec=service_timeout):
            raise RuntimeError("/compute_ik did not appear within timeout")
        node.get_logger().info("IKSolver: /compute_ik ready.")

    # ---------------------------------------------------------------- internal
    def _await(self, future) -> Optional[GetPositionIK.Response]:
        """Wait for a future without re-spinning the node (avoids executor deadlock)."""
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=self._call_timeout):
            return None
        return future.result()

    # ---------------------------------------------------------------- public
    def solve(self, x: float, y: float, z: float, surface: str,
              tcp_yaw: float = 0.0, tilt_rad: float = 0.0,
              tilt_axis: str = "x",
              avoid_collisions: bool = True,
              seed_joints: Optional[List[float]] = None) -> IKResult:
        """Solve IK for a single target. Returns IKResult.

        seed_joints (optional) — list of 6 angles in PIPER_JOINTS order. When
        supplied, the underlying KDL solver starts its numerical search from
        this state, so two adjacent calls with similar targets stay on the
        same IK branch (no elbow flips, no wrist sign changes). Critical for
        producing smooth joint trajectories from a coverage planner.
        """
        try:
            q = surface_quaternion(surface, tcp_yaw, tilt_rad, tilt_axis)
        except ValueError as exc:
            return IKResult(success=False, reason=str(exc))

        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.x = float(q[0])
        pose.pose.orientation.y = float(q[1])
        pose.pose.orientation.z = float(q[2])
        pose.pose.orientation.w = float(q[3])

        req = GetPositionIK.Request()
        req.ik_request.group_name = ARM_GROUP
        req.ik_request.ik_link_name = TIP_LINK
        req.ik_request.pose_stamped = pose
        req.ik_request.avoid_collisions = bool(avoid_collisions)
        req.ik_request.timeout.sec = int(self._ik_timeout)
        req.ik_request.timeout.nanosec = int((self._ik_timeout % 1.0) * 1e9)

        # Seed the KDL solver with the previous-waypoint joint state so the
        # numerical search converges to the nearest branch instead of flipping
        # elbow / wrist sign between adjacent waypoints.
        if seed_joints is not None and len(seed_joints) == len(PIPER_JOINTS):
            rs = RobotState()
            js = JointState()
            js.name = list(PIPER_JOINTS)
            js.position = [float(v) for v in seed_joints]
            rs.joint_state = js
            req.ik_request.robot_state = rs

        future = self._client.call_async(req)
        resp = self._await(future)
        if resp is None:
            return IKResult(success=False, reason="service_timeout")

        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            return IKResult(success=False, reason=_error_name(resp.error_code.val))

        name_to_pos = dict(zip(resp.solution.joint_state.name,
                               resp.solution.joint_state.position))
        angles = [float(name_to_pos.get(j, 0.0)) for j in PIPER_JOINTS]
        return IKResult(success=True, joint_angles=angles)


# Map a handful of MoveIt error codes to short, log-friendly names.
_ERROR_NAMES = {
    MoveItErrorCodes.SUCCESS: "success",
    MoveItErrorCodes.NO_IK_SOLUTION: "no_ik_solution",
    MoveItErrorCodes.TIMED_OUT: "timed_out",
    MoveItErrorCodes.GOAL_IN_COLLISION: "goal_in_collision",
    MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS: "goal_violates_constraints",
    MoveItErrorCodes.INVALID_GROUP_NAME: "invalid_group_name",
    MoveItErrorCodes.PLANNING_FAILED: "planning_failed",
    MoveItErrorCodes.FAILURE: "failure",
}


def _error_name(code: int) -> str:
    return _ERROR_NAMES.get(code, f"err_{code}")
