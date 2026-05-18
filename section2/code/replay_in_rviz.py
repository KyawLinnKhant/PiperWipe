#!/usr/bin/env python3
"""
replay_in_rviz.py — send Section 2's joint trajectories to the arm_controller
so the arm actually moves in RViz.

Reads outputs/trajectory_<surface>.csv, builds a trajectory_msgs/JointTrajectory
goal, and sends it to /arm_controller/follow_joint_trajectory. Both surfaces
are replayed in sequence with a short pause between them.

Run AFTER `ros2 launch piper_wiping kitchen_full_launch.py` is up.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import pandas as pd
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

PIPER_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

S2_OUT = Path(__file__).resolve().parents[1] / "outputs"


def _duration(t: float) -> DurationMsg:
    sec = int(t)
    return DurationMsg(sec=sec, nanosec=int((t - sec) * 1e9))


def build_goal(csv_path: Path, start_delay: float = 1.5) -> FollowJointTrajectory.Goal:
    """Read a Section 2 trajectory CSV and pack it into an action goal."""
    df = pd.read_csv(csv_path)

    traj = JointTrajectory()
    traj.joint_names = PIPER_JOINTS

    # Insert a "settle at first pose" point at t=settle_delay so the controller
    # has time to ramp from wherever the arm currently sits. The trajectory
    # itself starts at t = start_delay (must be > settle_delay).
    settle_delay = 0.5
    first_q = [float(df[j].iloc[0]) for j in PIPER_JOINTS]
    p0 = JointTrajectoryPoint()
    p0.positions = first_q
    p0.time_from_start = _duration(settle_delay)
    traj.points.append(p0)

    # Drop any duplicate t=0 first row by skipping it; ensure strict monotonic.
    last_t = settle_delay
    for row in df.itertuples():
        t = start_delay + float(row.time_s)
        if t <= last_t:
            t = last_t + 1e-3
        p = JointTrajectoryPoint()
        p.positions = [float(getattr(row, j)) for j in PIPER_JOINTS]
        p.time_from_start = _duration(t)
        traj.points.append(p)
        last_t = t

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj
    return goal


class ReplayClient(Node):
    def __init__(self):
        super().__init__("section2_rviz_replay")
        self._client = ActionClient(self, FollowJointTrajectory,
                                     "/arm_controller/follow_joint_trajectory")
        self.get_logger().info("Waiting for /arm_controller/follow_joint_trajectory …")
        self._client.wait_for_server()
        self.get_logger().info("Connected.")

    def send(self, csv_path: Path, label: str, start_delay: float = 1.5):
        goal = build_goal(csv_path, start_delay)
        n_pts = len(goal.trajectory.points)
        total = goal.trajectory.points[-1].time_from_start
        dur_s = total.sec + total.nanosec * 1e-9
        self.get_logger().info(
            f"[{label}] sending {n_pts} points, total {dur_s:.1f} s"
        )
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"[{label}] goal REJECTED")
            return
        self.get_logger().info(f"[{label}] goal accepted, awaiting execution …")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info(f"[{label}] execution complete")


def main():
    rclpy.init()
    client = ReplayClient()
    try:
        runs = [
            ("countertop", S2_OUT / "trajectory_countertop.csv"),
            ("mirror",     S2_OUT / "trajectory_mirror.csv"),
        ]
        for label, csv in runs:
            if not csv.exists():
                client.get_logger().warning(f"{csv} missing — skipping {label}")
                continue
            client.send(csv, label)
            time.sleep(2.0)   # short breather between surfaces
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
