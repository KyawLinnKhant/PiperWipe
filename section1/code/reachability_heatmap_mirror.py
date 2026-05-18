#!/usr/bin/env python3
"""
reachability_heatmap_mirror.py — Section 1 bonus deliverable for the mirror.

Mirrors the countertop heatmap: 60 × 90 cm panel @ 2 cm grid, same pitch × yaw
orientation lattice, same collision-aware IK. Output goes alongside the
countertop CSV/PNG and is consumed by Section 2's mirror semicircle planner as a mask.
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


# ── Patch geometry (mirror panel, world frame) ──────────────────────────────
PATCH_X = (-0.30, 0.30)        # 60 cm wide
PATCH_Z = (0.05, 0.95)         # 90 cm tall (matches the mirror box)
RES = 0.02                     # 2 cm grid

MIRROR_FRONT_FACE_Y = 0.555 - 0.005           # 0.55 m
GRIPPER_TIP_OFFSET = 0.1358
TIP_CLEARANCE = 0.005
Y_TARGET = MIRROR_FRONT_FACE_Y - (GRIPPER_TIP_OFFSET + TIP_CLEARANCE)   # 0.4092 m

# Same orientation lattice as the countertop heatmap
TRY_YAWS_DEG = [0, 30, -30, 60, -60, 90, -90, 120, -120, 150, -150, 180]
TRY_PITCHES_DEG = [0, 5, -5, 10, -10]

OUT_DIR = Path(__file__).resolve().parents[1] / "outputs"
OUT_DIR.mkdir(parents=True, exist_ok=True)


def cell_grid():
    xs = np.arange(PATCH_X[0], PATCH_X[1] + RES / 2, RES)
    zs = np.arange(PATCH_Z[0], PATCH_Z[1] + RES / 2, RES)
    return xs, zs


def sweep(node: Node, solver: IKSolver):
    xs, zs = cell_grid()
    total = len(xs) * len(zs)
    rows, done, t0 = [], 0, time.monotonic()

    for x in xs:
        for z in zs:
            best, reason = None, "no_ik_solution"
            for pitch_deg in TRY_PITCHES_DEG:
                for yaw_deg in TRY_YAWS_DEG:
                    r = solver.solve(
                        float(x), Y_TARGET, float(z),
                        surface="mirror",
                        tcp_yaw=math.radians(yaw_deg),
                        tilt_rad=math.radians(pitch_deg),
                    )
                    if r.success:
                        best, reason = (pitch_deg, yaw_deg), ""
                        break
                    reason = r.reason
                if best is not None:
                    break
            rows.append({
                "x": round(float(x), 3),
                "z": round(float(z), 3),
                "reachable": int(best is not None),
                "best_pitch_deg": best[0] if best is not None else "",
                "best_yaw_deg":   best[1] if best is not None else "",
                "reason": reason,
            })
            done += 1
            if done % 50 == 0:
                rate = done / max(time.monotonic() - t0, 1e-6)
                node.get_logger().info(
                    f"Mirror progress {done}/{total} "
                    f"({100 * done / total:.0f}%) — {rate:.1f} cell/s"
                )

    return pd.DataFrame(rows)


def render(df: pd.DataFrame, png_path: Path):
    pivot = df.pivot(index="z", columns="x", values="reachable").sort_index()
    fig, ax = plt.subplots(figsize=(9, 11))
    extent = [PATCH_X[0] - RES / 2, PATCH_X[1] + RES / 2,
              PATCH_Z[0] - RES / 2, PATCH_Z[1] + RES / 2]
    im = ax.imshow(pivot.values, cmap="RdYlGn", origin="lower",
                   extent=extent, aspect="equal", vmin=0, vmax=1)
    fig.colorbar(im, ax=ax, label="Reachable (1 = yes, 0 = no)")

    # Mirror panel outline
    ax.add_patch(patches.Rectangle((-0.30, 0.05), 0.60, 0.90,
                                   linewidth=1.5, edgecolor="purple",
                                   facecolor="none", linestyle="--",
                                   label="Mirror panel (0.6 × 0.9 m)"))

    pct = 100.0 * df["reachable"].sum() / len(df)
    ax.set_title(
        f"Piper reachability — mirror panel @ 2 cm "
        f"({df['reachable'].sum()}/{len(df)} cells reachable, {pct:.1f}%)\n"
        "Pitch ∈ {0°,±5°,±10°} × yaw ∈ {0°,±30°,…,180°}, collision-aware IK"
    )
    ax.set_xlabel("X (m, world) — horizontal")
    ax.set_ylabel("Z (m, world) — vertical")
    ax.set_xlim(PATCH_X[0] - 0.05, PATCH_X[1] + 0.05)
    ax.set_ylim(PATCH_Z[0] - 0.05, PATCH_Z[1] + 0.05)
    ax.grid(True, linestyle=":", alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)
    plt.tight_layout()
    plt.savefig(png_path, dpi=150)
    plt.close(fig)


def main(argv=None):
    rclpy.init(args=argv)
    node = Node("reachability_heatmap_mirror")
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    import threading
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        solver = IKSolver(node)
        df = sweep(node, solver)
        csv_path = OUT_DIR / "reachability_mirror.csv"
        png_path = OUT_DIR / "reachability_mirror_heatmap.png"
        df.to_csv(csv_path, index=False)
        node.get_logger().info(f"Wrote {csv_path}")
        render(df, png_path)
        node.get_logger().info(f"Wrote {png_path}")
        reachable = int(df["reachable"].sum())
        node.get_logger().info(
            f"Mirror summary: {reachable}/{len(df)} cells "
            f"({100.0 * reachable / len(df):.1f}%) reachable."
        )
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
