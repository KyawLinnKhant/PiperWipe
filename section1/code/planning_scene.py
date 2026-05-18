#!/usr/bin/env python3
"""
planning_scene.py — Section 1 planning-scene publisher.

kitchen_scene.py (in piper_wiping) publishes visual RViz markers only — those
are display-only and have no effect on collision checks. This node pushes the
same objects (countertop slab, mirror, faucet base + spout) into MoveIt's
PlanningScene as CollisionObjects so IK / planning can reject infeasible
solutions that intersect the environment.

Coordinates are kept in lock-step with kitchen_scene.py (see positions.md).
Run any time after move_group is up:

    python3 planning_scene.py

Publishes diffs on /planning_scene (queue 1, latched) every second so a late
RViz/IK client always sees the scene without needing /apply_planning_scene.
"""

from dataclasses import dataclass
from typing import Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneWorld
from shape_msgs.msg import SolidPrimitive


@dataclass
class Box:
    name: str
    xyz: Tuple[float, float, float]
    size: Tuple[float, float, float]


@dataclass
class Cylinder:
    name: str
    xyz: Tuple[float, float, float]
    height: float
    radius: float


KITCHEN_BOXES = [
    Box("countertop",    (0.00, 0.25,  0.025), (1.20, 0.60, 0.05)),
    Box("mirror",        (0.00, 0.555, 0.500), (0.60, 0.01, 0.90)),
    Box("faucet_spout",  (0.00, 0.475, 0.157), (0.015, 0.05, 0.015)),
]
KITCHEN_CYLINDERS = [
    Cylinder("faucet_base", (0.00, 0.50, 0.10), height=0.10, radius=0.015),
]


def _pose(xyz: Tuple[float, float, float]) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in xyz)
    p.orientation.w = 1.0
    return p


def _box_collision(box: Box) -> CollisionObject:
    co = CollisionObject()
    co.id = box.name
    co.header.frame_id = "world"
    prim = SolidPrimitive()
    prim.type = SolidPrimitive.BOX
    prim.dimensions = list(box.size)
    co.primitives = [prim]
    co.primitive_poses = [_pose(box.xyz)]
    co.operation = CollisionObject.ADD
    return co


def _cyl_collision(cyl: Cylinder) -> CollisionObject:
    co = CollisionObject()
    co.id = cyl.name
    co.header.frame_id = "world"
    prim = SolidPrimitive()
    prim.type = SolidPrimitive.CYLINDER
    prim.dimensions = [float(cyl.height), float(cyl.radius)]
    co.primitives = [prim]
    co.primitive_poses = [_pose(cyl.xyz)]
    co.operation = CollisionObject.ADD
    return co


class PlanningSceneBroadcaster(Node):
    def __init__(self):
        super().__init__("planning_scene_publisher")
        latched = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(PlanningScene, "/planning_scene", latched)
        self._scene = self._build_scene()
        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            f"Publishing {len(KITCHEN_BOXES) + len(KITCHEN_CYLINDERS)} "
            "collision objects to /planning_scene"
        )

    def _build_scene(self) -> PlanningScene:
        world = PlanningSceneWorld()
        world.collision_objects = (
            [_box_collision(b) for b in KITCHEN_BOXES]
            + [_cyl_collision(c) for c in KITCHEN_CYLINDERS]
        )
        scene = PlanningScene()
        scene.is_diff = True
        scene.world = world
        return scene

    def _tick(self):
        self._pub.publish(self._scene)


def main(argv=None):
    rclpy.init(args=argv)
    node = PlanningSceneBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
