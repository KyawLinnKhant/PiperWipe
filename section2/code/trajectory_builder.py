"""
trajectory_builder.py — Section 2 deliverable 2.

Walks a CoveragePlan, calls Section 1's IKSolver on every waypoint, and
time-parameterizes the resulting joint trajectory at a constant Cartesian
speed (each surface gets the spec-defined nominal speed).

Output:
  * trajectory_msgs/JointTrajectory.msg-like dict (Python) for ROS hand-off.
  * CSV  (time_s, joint1..joint6, x, y, z, surface)  for plotting + handoff.
  * In-place population of plan.path_length_m and plan.duration_s.

Waypoints whose IK fails are SKIPPED with a warning — they show up as
missing rows in the CSV so the visualizer can mark unwiped gaps.
"""

from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List

import numpy as np
import pandas as pd

# Import Section 1's solver
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "section1" / "code"))
from ik_solver import IKSolver, PIPER_JOINTS  # noqa: E402

from coverage_planner import CoveragePlan, path_length  # noqa: E402


SPEED = {"countertop": 0.20, "mirror": 0.15}   # m/s — within spec ranges


@dataclass
class TrajectoryPoint:
    t: float
    q: List[float]      # joint angles, in PIPER_JOINTS order
    x: float
    y: float
    z: float
    pitch_deg: float
    yaw_deg: float


def build_trajectory(plan: CoveragePlan, solver: IKSolver,
                     pitch_yaw_lattice: List[tuple] = None) -> List[TrajectoryPoint]:
    """Solve IK + time-parameterize. Updates plan.path_length_m / duration_s."""
    if pitch_yaw_lattice is None:
        # Cheap first, lean later — matches the Section 1 heatmap order.
        pitch_yaw_lattice = [
            (0, 0), (0, 30), (0, -30), (0, 60), (0, -60), (0, 90), (0, -90),
            (5, 0), (5, 30), (5, -30), (-5, 0), (-5, 30), (-5, -30),
            (10, 0), (10, 30), (10, -30), (-10, 0), (-10, 30), (-10, -30),
        ]

    speed = SPEED.get(plan.surface, 0.15)
    pts: List[TrajectoryPoint] = []
    t_accum = 0.0
    last_xyz = None
    last_q = None        # previous solved joints — seed for IK continuity
    n_solved = 0

    for i, wp in enumerate(plan.waypoints):
        solved = None
        for pitch_deg, yaw_deg in pitch_yaw_lattice:
            r = solver.solve(
                wp.x, wp.y, wp.z,
                surface=wp.surface,
                tcp_yaw=math.radians(yaw_deg),
                tilt_rad=math.radians(pitch_deg),
                seed_joints=last_q,    # keep solver on the same IK branch
            )
            if r.success:
                solved = (pitch_deg, yaw_deg, r.joint_angles)
                break
        if solved is None:
            continue  # skip this waypoint; visualizer will show the gap
        pitch_deg, yaw_deg, q = solved
        last_q = q
        if last_xyz is not None:
            seg = math.dist(last_xyz, (wp.x, wp.y, wp.z))
            t_accum += seg / speed
        pts.append(TrajectoryPoint(
            t=t_accum, q=q, x=wp.x, y=wp.y, z=wp.z,
            pitch_deg=float(pitch_deg), yaw_deg=float(yaw_deg),
        ))
        last_xyz = (wp.x, wp.y, wp.z)
        n_solved += 1
        if (i + 1) % 50 == 0:
            print(f"  [{plan.surface}] {i+1}/{len(plan.waypoints)} planned "
                  f"({n_solved} solved)")

    plan.path_length_m = path_length(plan)
    plan.duration_s = pts[-1].t if pts else 0.0
    return pts


def write_csv(pts: List[TrajectoryPoint], out_path: Path, surface: str) -> None:
    rows = []
    for p in pts:
        row = {"time_s": round(p.t, 4),
               "x": round(p.x, 5), "y": round(p.y, 5), "z": round(p.z, 5),
               "surface": surface,
               "pitch_deg": p.pitch_deg, "yaw_deg": p.yaw_deg}
        for name, val in zip(PIPER_JOINTS, p.q):
            row[name] = round(float(val), 6)
        rows.append(row)
    pd.DataFrame(rows).to_csv(out_path, index=False)


def joint_trajectory_dict(pts: List[TrajectoryPoint]) -> dict:
    """Return a trajectory_msgs/JointTrajectory-shaped dict for direct
    YAML/JSON dump or rclpy publishing."""
    return {
        "joint_names": PIPER_JOINTS,
        "points": [
            {
                "positions": [float(v) for v in p.q],
                "time_from_start": {"sec": int(p.t),
                                     "nanosec": int((p.t % 1.0) * 1e9)},
            }
            for p in pts
        ],
    }
