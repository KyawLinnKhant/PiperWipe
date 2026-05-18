#!/usr/bin/env python3
"""
kitchen_scene.py
Publishes kitchen environment markers and locks the gripper at 50 mm opening.

All world-frame positions from positions.md (confirmed 2026-05-17).
Sponge is published in the 'gripper_base' TF frame so it moves with the arm.
All marker stamps are zero so RViz uses the latest available TF (avoids
TF timestamp mismatches that cause flickering when using non-world frames).
"""

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray


# Gripper open 50 mm total: each finger 25 mm from centre
JOINT7 =  0.025   # m  (finger 1)
JOINT8 = -0.025   # m  (finger 2)


def _stamp_zero() -> TimeMsg:
    """Return a zero ROS stamp — tells RViz to use the latest available TF."""
    return TimeMsg(sec=0, nanosec=0)


class KitchenScene(Node):
    def __init__(self):
        super().__init__('kitchen_scene')
        self.markers_pub  = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.gripper_pub  = self.create_publisher(JointState,  '/gripper_joint_states', 10)
        self.create_timer(0.1, self._publish)
        self.get_logger().info('KitchenScene node started.')

    # ------------------------------------------------------------------ helpers

    def _box(self, uid, frame, x, y, z, sx, sy, sz, r, g, b, a=1.0) -> Marker:
        m = Marker()
        m.header.frame_id      = frame
        m.header.stamp         = _stamp_zero()
        m.ns                   = 'kitchen'
        m.id                   = uid
        m.type                 = Marker.CUBE
        m.action               = Marker.ADD
        m.pose.position.x      = float(x)
        m.pose.position.y      = float(y)
        m.pose.position.z      = float(z)
        m.pose.orientation.w   = 1.0
        m.scale.x, m.scale.y, m.scale.z = float(sx), float(sy), float(sz)
        m.color.r, m.color.g, m.color.b, m.color.a = float(r), float(g), float(b), float(a)
        return m

    def _cyl(self, uid, x, y, z, radius, height, r, g, b, a=1.0) -> Marker:
        m = Marker()
        m.header.frame_id      = 'world'
        m.header.stamp         = _stamp_zero()
        m.ns                   = 'kitchen'
        m.id                   = uid
        m.type                 = Marker.CYLINDER
        m.action               = Marker.ADD
        m.pose.position.x      = float(x)
        m.pose.position.y      = float(y)
        m.pose.position.z      = float(z)
        m.pose.orientation.w   = 1.0
        m.scale.x = m.scale.y  = float(radius * 2)
        m.scale.z              = float(height)
        m.color.r, m.color.g, m.color.b, m.color.a = float(r), float(g), float(b), float(a)
        return m

    def _arrow(self, uid, dx, dy, dz, r, g, b, length=0.20) -> Marker:
        m = Marker()
        m.header.frame_id    = 'world'
        m.header.stamp       = _stamp_zero()
        m.ns                 = 'axes'
        m.id                 = uid
        m.type               = Marker.ARROW
        m.action             = Marker.ADD
        m.points             = [Point(x=0.0, y=0.0, z=0.0),
                                 Point(x=float(dx*length), y=float(dy*length), z=float(dz*length))]
        m.scale.x            = 0.008   # shaft diameter
        m.scale.y            = 0.016   # head diameter
        m.scale.z            = 0.024   # head length
        m.color.r, m.color.g, m.color.b, m.color.a = float(r), float(g), float(b), 1.0
        return m

    # ------------------------------------------------------------------ publish

    def _publish(self):
        ma = MarkerArray()
        ma.markers = [
            # countertop — walnut brown (positions.md: 0.00, 0.25, 0.025 / 1.20×0.60×0.05)
            self._box(0, 'world', 0.00, 0.25,  0.025, 1.20, 0.60, 0.05, 0.42, 0.22, 0.07),
            # mirror — transparent blue (0.00, 0.555, 0.50 / 0.60×0.01×0.90)
            self._box(1, 'world', 0.00, 0.555, 0.50,  0.60, 0.01, 0.90, 0.78, 0.94, 1.00, 0.18),
            # faucet base — brushed steel (0.00, 0.50, 0.10)
            self._cyl(2,          0.00, 0.50,  0.10,  0.015, 0.10, 0.80, 0.82, 0.86),
            # faucet spout — brushed steel (0.00, 0.475, 0.157 / 0.015×0.05×0.015)
            self._box(3, 'world', 0.00, 0.475, 0.157, 0.015, 0.05, 0.015, 0.80, 0.82, 0.86),
            # sponge — yellow, in gripper_base frame so it tracks the arm
            # z=0.130 puts sponge at gripper finger level (joint7/8 origins at z=0.1358)
            # long axis (0.070) in gripper_base X = perpendicular to +Y arm reach
            # y=0.038 in finger closing direction, fits inside 50 mm gap (±0.025)
            self._box(4, 'gripper_base', 0.0, 0.0, 0.130, 0.070, 0.038, 0.020, 1.0, 1.0, 0.0),
            # world-frame axes
            self._arrow(10, 1, 0, 0, 1.0, 0.0, 0.0),   # +X red
            self._arrow(11, 0, 1, 0, 0.0, 1.0, 0.0),   # +Y green
            self._arrow(12, 0, 0, 1, 0.0, 0.0, 1.0),   # +Z blue
        ]
        self.markers_pub.publish(ma)

        # Lock gripper fingers at 50 mm opening (25 mm each side)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name     = ['joint7', 'joint8']
        js.position = [JOINT7, JOINT8]
        self.gripper_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = KitchenScene()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
