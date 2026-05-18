#!/usr/bin/env python3
"""
reachability_heatmap.py — Section 1 deliverable 2.

Sweeps a 60×60 cm patch of the countertop at 2 cm resolution and, for each
cell, asks IKSolver whether the tool can reach a surface-aligned pose just
above that cell.

To match a competent real-world planner, each cell is retried with a small
(pitch × yaw) lattice — yaw spins the gripper about the tool's approach axis
(swaps which gripper face points at the mirror/faucet) and pitch tilts the
tool away from the surface normal by up to ±10° (within the assignment's
surface-normal tolerance, lets the wrist slip past the faucet body).

Output:
    outputs/reachability.csv         per-cell (x, y, reachable, best_pitch_deg,
                                              best_yaw_deg, reason)
    outputs/reachability_heatmap.png 2-D heatmap with arm base + faucet overlay

Geometry (consistent with kitchen_scene.py):
    countertop  centre (0.00, 0.25, 0.025), size 1.20 × 0.60 × 0.05  → top at z=0.05
    arm base    (0.00, 0.05, 0.05)
    faucet base cylinder at (0.00, 0.50), r=0.015

Run after MoveIt + planning_scene.py are up:
    python3 reachability_heatmap.py
"""

import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from ik_solver import IKSolver


# ── Patch geometry ────────────────────────────────────────────────────────────
PATCH_X = (-0.30, 0.30)        # 60 cm wide, centred on the arm base in X
PATCH_Y = (-0.05, 0.55)        # 60 cm deep, spanning the full countertop in Y
RES = 0.02                     # 2 cm grid

# IK targets the "arm" group's tip_link (= link6, the wrist). The gripper tip
# is GRIPPER_TIP_OFFSET below link6 along the tool's +Z. So to put the tool tip
# CLEARANCE above the countertop top face, we aim link6 at (top + offset + clr).
COUNTERTOP_TOP_Z = 0.05
GRIPPER_TIP_OFFSET = 0.1358    # link6 → finger origin (joint7/8 origin in gripper_base)
TIP_CLEARANCE = 0.005          # 5 mm of breathing room above the surface
Z_TARGET = COUNTERTOP_TOP_Z + GRIPPER_TIP_OFFSET + TIP_CLEARANCE  # = 0.1908 m

# Orientation search lattice — first success wins. Yaws cover a full circle in
# 30° steps (gripper spin about the approach axis); pitches stay within the
# assignment's ±10° surface-normal tolerance. Order matters: cheap centred
# orientations are tried first so most cells solve in 1-2 calls.
TRY_YAWS_DEG = [0, 30, -30, 60, -60, 90, -90, 120, -120, 150, -150, 180]
TRY_PITCHES_DEG = [0, 5, -5, 10, -10]

OUT_DIR = Path(__file__).resolve().parents[1] / "outputs"
OUT_DIR.mkdir(parents=True, exist_ok=True)


def cell_grid():
    xs = np.arange(PATCH_X[0], PATCH_X[1] + RES / 2, RES)
    ys = np.arange(PATCH_Y[0], PATCH_Y[1] + RES / 2, RES)
    return xs, ys


def sweep(node: Node, solver: IKSolver):
    xs, ys = cell_grid()
    total = len(xs) * len(ys)
    rows = []
    done = 0
    t0 = time.monotonic()

    for x in xs:
        for y in ys:
            best = None  # (pitch_deg, yaw_deg)
            reason = "no_ik_solution"
            # Pitch outer / yaw inner so the planner first tries straight-down
            # with the full yaw circle (cheapest realistic poses), then leans.
            for pitch_deg in TRY_PITCHES_DEG:
                for yaw_deg in TRY_YAWS_DEG:
                    r = solver.solve(
                        float(x), float(y), Z_TARGET,
                        surface="countertop",
                        tcp_yaw=math.radians(yaw_deg),
                        tilt_rad=math.radians(pitch_deg),
                    )
                    if r.success:
                        best = (pitch_deg, yaw_deg)
                        reason = ""
                        break
                    reason = r.reason
                if best is not None:
                    break
            rows.append({
                "x": round(float(x), 3),
                "y": round(float(y), 3),
                "reachable": int(best is not None),
                "best_pitch_deg": best[0] if best is not None else "",
                "best_yaw_deg":   best[1] if best is not None else "",
                "reason": reason,
            })
            done += 1
            if done % 50 == 0:
                rate = done / max(time.monotonic() - t0, 1e-6)
                node.get_logger().info(
                    f"Progress {done}/{total} ({100 * done / total:.0f}%) "
                    f"— {rate:.1f} cell/s"
                )

    return pd.DataFrame(rows)


def render(df: pd.DataFrame, png_path: Path):
    pivot = df.pivot(index="y", columns="x", values="reachable").sort_index()

    fig, ax = plt.subplots(figsize=(10, 9))
    extent = [PATCH_X[0] - RES / 2, PATCH_X[1] + RES / 2,
              PATCH_Y[0] - RES / 2, PATCH_Y[1] + RES / 2]
    im = ax.imshow(pivot.values, cmap="RdYlGn", origin="lower",
                   extent=extent, aspect="equal", vmin=0, vmax=1)
    fig.colorbar(im, ax=ax, label="Reachable (1 = yes, 0 = no)")

    # Countertop outline (1.2 × 0.6 m, centred on x=0)
    counter = patches.Rectangle((-0.60, -0.05), 1.20, 0.60,
                                linewidth=1.5, edgecolor="black",
                                facecolor="none", linestyle="--",
                                label="Countertop (1.2 × 0.6 m)")
    ax.add_patch(counter)

    # Arm base
    ax.plot(0.00, 0.05, marker="^", markersize=14, color="black",
            label="Arm base (0, 0.05)")

    # Faucet (radius 1.5 cm at (0, 0.50)) — also bake in a small clearance buffer
    faucet = patches.Circle((0.00, 0.50), 0.015,
                            linewidth=2, edgecolor="blue",
                            facecolor="none", label="Faucet (r = 15 mm)")
    ax.add_patch(faucet)

    # Mirror back edge
    ax.axhline(0.555, color="purple", linestyle=":", linewidth=1.5,
               label="Mirror face (y = 0.555)")

    pct = 100.0 * df["reachable"].sum() / len(df)
    ax.set_title(
        f"Piper reachability — 60×60 cm countertop patch @ 2 cm "
        f"({df['reachable'].sum()}/{len(df)} cells reachable, {pct:.1f}%)\n"
        "Pitch ∈ {0°,±5°,±10°} × yaw ∈ {0°,±30°,…,180°}, collision-aware IK"
    )
    ax.set_xlabel("X  (m, world)")
    ax.set_ylabel("Y  (m, world)")
    ax.set_xlim(PATCH_X[0] - 0.05, PATCH_X[1] + 0.05)
    ax.set_ylim(PATCH_Y[0] - 0.05, PATCH_Y[1] + 0.05)
    ax.grid(True, linestyle=":", alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)
    plt.tight_layout()
    plt.savefig(png_path, dpi=150)
    plt.close(fig)


def main(argv=None):
    rclpy.init(args=argv)
    node = Node("reachability_heatmap")
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    import threading
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        solver = IKSolver(node)
        df = sweep(node, solver)

        csv_path = OUT_DIR / "reachability.csv"
        png_path = OUT_DIR / "reachability_heatmap.png"
        df.to_csv(csv_path, index=False)
        node.get_logger().info(f"Wrote {csv_path}")
        render(df, png_path)
        node.get_logger().info(f"Wrote {png_path}")

        reachable = int(df["reachable"].sum())
        node.get_logger().info(
            f"Summary: {reachable}/{len(df)} cells "
            f"({100.0 * reachable / len(df):.1f}%) reachable."
        )
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
