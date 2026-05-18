#!/usr/bin/env python3
"""
ik_service.py — Section 1 standalone ROS 2 node.

Exposes IKSolver as a real ROS 2 service so the IK functionality is reachable
from any client (Python or C++) without going through MoveIt's PlanningScene
API directly. The service type is the standard moveit_msgs/srv/GetPositionIK,
so no custom .srv files / message generation are required.

Behavior:
  * Re-validates the request (group must be "arm").
  * Forwards to the underlying /compute_ik with avoid_collisions=True so that
    the planning scene (countertop, mirror, faucet — published separately) can
    cause infeasible poses to be rejected.
  * Returns the response unchanged.

Run (after sourcing the workspace and launching MoveIt + the planning scene):
    python3 ik_service.py
"""

import sys
from pathlib import Path

# Allow `python3 ik_service.py` to find ik_solver.py sitting next to it.
sys.path.insert(0, str(Path(__file__).resolve().parent))

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK

from ik_solver import ARM_GROUP, IKSolver


class IKServiceNode(Node):
    def __init__(self):
        super().__init__("ik_service")
        cb = ReentrantCallbackGroup()
        self._solver = IKSolver(self)
        self._srv = self.create_service(
            GetPositionIK, "/solve_ik", self._handle, callback_group=cb
        )
        self.get_logger().info("IK service ready on /solve_ik (moveit_msgs/GetPositionIK)")

    def _handle(self, request: GetPositionIK.Request,
                response: GetPositionIK.Response) -> GetPositionIK.Response:
        # The request is forwarded verbatim — but we sanity-check the group so
        # bad calls fail loudly with a clear MoveIt error code.
        if request.ik_request.group_name and request.ik_request.group_name != ARM_GROUP:
            response.error_code.val = MoveItErrorCodes.INVALID_GROUP_NAME
            return response

        request.ik_request.group_name = ARM_GROUP
        request.ik_request.avoid_collisions = True

        future = self._solver._client.call_async(request)  # noqa: SLF001
        resp = self._solver._await(future)                  # noqa: SLF001
        if resp is None:
            response.error_code.val = MoveItErrorCodes.TIMED_OUT
            return response
        return resp


def main(argv=None):
    rclpy.init(args=argv)
    node = IKServiceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
