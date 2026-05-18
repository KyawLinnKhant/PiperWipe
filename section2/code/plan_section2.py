#!/usr/bin/env python3
"""
plan_section2.py — orchestrator for Section 2 deliverables.

Runs both coverage strategies, solves IK on every waypoint, time-parameterises,
dumps trajectory CSVs + JointTrajectory YAML, generates matplotlib plots,
prints metrics.

Prerequisites: MoveIt up + planning_scene.py running (see Section 1 README).
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import rclpy
import yaml
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "section1" / "code"))   # ik_solver
sys.path.insert(0, str(HERE))                                    # local imports

from ik_solver import IKSolver  # noqa: E402

from coverage_planner import (  # noqa: E402
    coverage_fraction, raster_countertop, mirror_wipe,
)
from trajectory_builder import (  # noqa: E402
    SPEED, build_trajectory, joint_trajectory_dict, write_csv,
)
from visualize_paths import (  # noqa: E402
    plot_countertop_raster, plot_joint_trajectories, plot_metrics_bar,
    plot_mirror_arcs,
)

OUT_DIR = HERE.parents[1] / "section2" / "outputs"
OUT_DIR.mkdir(parents=True, exist_ok=True)

REACH_CSV = HERE.parents[1] / "section1" / "outputs" / "reachability.csv"
REACH_MIRROR_CSV = HERE.parents[1] / "section1" / "outputs" / "reachability_mirror.csv"


def main():
    import threading

    rclpy.init()
    node = Node("section2_planner")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        solver = IKSolver(node)

        # ── 1. Countertop raster ─────────────────────────────────────────
        node.get_logger().info("Planning countertop raster …")
        plan_c = raster_countertop()
        node.get_logger().info(
            f"  generated {len(plan_c.waypoints)} waypoints (after mask + faucet keep-out)"
        )
        pts_c = build_trajectory(plan_c, solver)
        write_csv(pts_c, OUT_DIR / "trajectory_countertop.csv", "countertop")

        cov_c = coverage_fraction(plan_c) * 100.0
        node.get_logger().info(
            f"  countertop coverage {cov_c:.1f}%, "
            f"path {plan_c.path_length_m:.2f} m, "
            f"{plan_c.duration_s:.1f} s @ {SPEED['countertop']} m/s")

        # ── 2. Mirror hybrid wipe (top raster + side verticals + roundabout) ──
        node.get_logger().info("Planning mirror hybrid wipe …")
        plan_m = mirror_wipe()
        node.get_logger().info(
            f"  generated {len(plan_m.waypoints)} waypoints (after edge keep-out)"
        )
        pts_m = build_trajectory(plan_m, solver)
        write_csv(pts_m, OUT_DIR / "trajectory_mirror.csv", "mirror")

        cov_m = coverage_fraction(plan_m) * 100.0
        node.get_logger().info(
            f"  mirror coverage {cov_m:.1f}%, "
            f"path {plan_m.path_length_m:.2f} m, "
            f"{plan_m.duration_s:.1f} s @ {SPEED['mirror']} m/s")

        # ── 3. JointTrajectory YAML (one per surface) ────────────────────
        for surface, pts in (("countertop", pts_c), ("mirror", pts_m)):
            yaml_path = OUT_DIR / f"joint_trajectory_{surface}.yaml"
            with yaml_path.open("w") as f:
                yaml.safe_dump(joint_trajectory_dict(pts), f, sort_keys=False)

        # ── 4. Metrics ────────────────────────────────────────────────────
        metrics = {
            "countertop": {
                "waypoints":      len(plan_c.waypoints),
                "ik_solved":      len(pts_c),
                "coverage_pct":   cov_c,
                "path_length_m":  plan_c.path_length_m,
                "duration_s":     plan_c.duration_s,
                "speed_mps":      SPEED["countertop"],
            },
            "mirror": {
                "waypoints":      len(plan_m.waypoints),
                "ik_solved":      len(pts_m),
                "coverage_pct":   cov_m,
                "path_length_m":  plan_m.path_length_m,
                "duration_s":     plan_m.duration_s,
                "speed_mps":      SPEED["mirror"],
            },
        }
        with (OUT_DIR / "metrics.json").open("w") as f:
            json.dump(metrics, f, indent=2)
        node.get_logger().info(f"metrics → {OUT_DIR / 'metrics.json'}")

        # ── 5. Visualizations ────────────────────────────────────────────
        plot_countertop_raster(
            plan_c, OUT_DIR / "countertop_raster.png",
            reach_csv=REACH_CSV,
            traj_csv=OUT_DIR / "trajectory_countertop.csv",
            coverage_pct=cov_c,
        )
        plot_mirror_arcs(
            plan_m, OUT_DIR / "mirror_arcs.png",
            reach_csv=REACH_MIRROR_CSV,
            traj_csv=OUT_DIR / "trajectory_mirror.csv",
            coverage_pct=cov_m,
        )
        plot_joint_trajectories(
            {"countertop": OUT_DIR / "trajectory_countertop.csv",
             "mirror":     OUT_DIR / "trajectory_mirror.csv"},
            OUT_DIR / "joint_trajectories.png",
        )
        plot_metrics_bar(metrics, OUT_DIR / "coverage_metrics.png")
        node.get_logger().info(f"plots → {OUT_DIR}")

    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
