"""
visualize_paths.py — Section 2 matplotlib visualizations.

Renders four PNGs:
  * countertop_raster.png   — top-down view with reach mask, pad swaths, path
  * mirror_arcs.png         — front-on view of mirror stacked half-circles
  * joint_trajectory.png    — 6 joint angles vs time, both surfaces
  * coverage_metrics.png    — bar chart of coverage %, path length, time
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from coverage_planner import (
    COUNTERTOP_X, COUNTERTOP_Y, FAUCET_KEEPOUT_R, FAUCET_XY,
    MIRROR_X, MIRROR_Z, PAD_LONG, PAD_SHORT,
    CoveragePlan,
)


def _draw_pad_swaths(ax, polys, alpha=0.20, color="#1f9d55"):
    for poly in polys[::3]:  # every 3rd polygon for legibility
        rect = patches.Polygon(
            poly, closed=True, facecolor=color, edgecolor="none",
            alpha=alpha, linewidth=0,
        )
        ax.add_patch(rect)


def _solved_xy_set(traj_csv: Path) -> set:
    """Return the set of (x, y, z) tuples (rounded to mm) that IK solved."""
    if traj_csv is None or not traj_csv.exists() or traj_csv.stat().st_size < 10:
        return set()
    df = pd.read_csv(traj_csv)
    return {(round(r.x, 3), round(r.y, 3), round(r.z, 3))
            for r in df.itertuples()}


def plot_countertop_raster(plan: CoveragePlan, out_path: Path,
                            reach_csv: Path = None,
                            traj_csv: Path = None,
                            coverage_pct: float = None) -> None:
    fig, ax = plt.subplots(figsize=(12, 8))

    # 1. Reachability mask in the background
    if reach_csv is not None and reach_csv.exists():
        df = pd.read_csv(reach_csv)
        xs = np.array(sorted(df["x"].unique()))
        ys = np.array(sorted(df["y"].unique()))
        pivot = df.pivot(index="y", columns="x", values="reachable").reindex(
            index=ys, columns=xs).to_numpy()
        ax.imshow(pivot, cmap="RdYlGn", origin="lower", alpha=0.18,
                  extent=[xs.min() - 0.01, xs.max() + 0.01,
                          ys.min() - 0.01, ys.max() + 0.01],
                  aspect="equal", vmin=0, vmax=1,
                  zorder=0, interpolation="nearest")

    # 2. Countertop outline
    ax.add_patch(patches.Rectangle(
        (COUNTERTOP_X[0], COUNTERTOP_Y[0]),
        COUNTERTOP_X[1] - COUNTERTOP_X[0],
        COUNTERTOP_Y[1] - COUNTERTOP_Y[0],
        fill=False, edgecolor="black", linestyle="--", linewidth=1.5,
        zorder=1, label="Countertop (1.2 × 0.6 m)",
    ))

    # 3. Faucet keep-out
    ax.add_patch(patches.Circle(
        FAUCET_XY, FAUCET_KEEPOUT_R, fill=True, alpha=0.25,
        facecolor="blue", edgecolor="blue", linewidth=1.5, zorder=2,
        label=f"Faucet keep-out (r = {FAUCET_KEEPOUT_R*1000:.0f} mm)",
    ))

    # 4. Pad swaths (every 3rd one)
    _draw_pad_swaths(ax, plan.swath_polys)

    # 5. Centre path — coloured by IK success
    solved = _solved_xy_set(traj_csv)
    if plan.waypoints:
        xs_p = np.array([w.x for w in plan.waypoints])
        ys_p = np.array([w.y for w in plan.waypoints])
        zs_p = np.array([w.z for w in plan.waypoints])
        if solved:
            success = np.array([(round(x, 3), round(y, 3), round(z, 3)) in solved
                                for x, y, z in zip(xs_p, ys_p, zs_p)])
            # Plot solved segments in dark green, skipped in red.
            for i in range(len(xs_p) - 1):
                col = "#1f9d55" if (success[i] and success[i + 1]) else "#c0392b"
                lw = 1.1 if (success[i] and success[i + 1]) else 0.7
                ax.plot(xs_p[i:i + 2], ys_p[i:i + 2], "-",
                        color=col, linewidth=lw, alpha=0.85, zorder=3)
            n_ok = int(success.sum())
            ax.plot([], [], "-", color="#1f9d55", linewidth=1.5,
                    label=f"Path — IK solved ({n_ok}/{len(success)})")
            ax.plot([], [], "-", color="#c0392b", linewidth=1.5,
                    label=f"Path — IK failed ({len(success) - n_ok}/{len(success)})")
        else:
            ax.plot(xs_p, ys_p, "-", color="#1f3d5c", linewidth=0.9,
                    alpha=0.75, zorder=3, label="Tool centre path")
        ax.plot(xs_p[0], ys_p[0], "o", color="green", markersize=10,
                zorder=4, label=f"Start ({xs_p[0]:+.2f}, {ys_p[0]:+.2f})")
        ax.plot(xs_p[-1], ys_p[-1], "s", color="red", markersize=10,
                zorder=4, label=f"End ({xs_p[-1]:+.2f}, {ys_p[-1]:+.2f})")

    # 5. Arm base
    ax.plot(0.0, 0.05, "^", color="black", markersize=14, zorder=5,
            label="Arm base (0, 0.05)")

    title = (f"Countertop raster — {len(plan.waypoints)} waypoints, "
             f"path {plan.path_length_m:.2f} m, "
             f"{plan.duration_s:.1f} s @ 0.20 m/s")
    if coverage_pct is not None:
        title += f"\nCoverage of reachable patch: {coverage_pct:.1f}%"
    ax.set_title(title)
    ax.set_xlabel("X (m, world)")
    ax.set_ylabel("Y (m, world)")
    ax.set_xlim(COUNTERTOP_X[0] - 0.05, COUNTERTOP_X[1] + 0.05)
    ax.set_ylim(COUNTERTOP_Y[0] - 0.05, COUNTERTOP_Y[1] + 0.05)
    ax.set_aspect("equal")
    ax.grid(True, linestyle=":", alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_mirror_arcs(plan: CoveragePlan, out_path: Path,
                       reach_csv: Path = None,
                       traj_csv: Path = None,
                       coverage_pct: float = None) -> None:
    fig, ax = plt.subplots(figsize=(9, 11))

    # Reachability mask in the background (if available)
    if reach_csv is not None and reach_csv.exists():
        df = pd.read_csv(reach_csv)
        xs = np.array(sorted(df["x"].unique()))
        zs = np.array(sorted(df["z"].unique()))
        pivot = df.pivot(index="z", columns="x", values="reachable").reindex(
            index=zs, columns=xs).to_numpy()
        ax.imshow(pivot, cmap="RdYlGn", origin="lower", alpha=0.18,
                  extent=[xs.min() - 0.01, xs.max() + 0.01,
                          zs.min() - 0.01, zs.max() + 0.01],
                  aspect="equal", vmin=0, vmax=1,
                  zorder=0, interpolation="nearest")

    # Mirror outline
    ax.add_patch(patches.Rectangle(
        (MIRROR_X[0], MIRROR_Z[0]),
        MIRROR_X[1] - MIRROR_X[0],
        MIRROR_Z[1] - MIRROR_Z[0],
        fill=False, edgecolor="purple", linestyle="--", linewidth=1.5,
        zorder=1, label="Mirror (0.6 × 0.9 m)",
    ))

    # Pad swaths
    for poly in plan.swath_polys[::3]:
        ax.add_patch(patches.Polygon(
            poly, closed=True, facecolor="#3e9fea", edgecolor="none",
            alpha=0.22, zorder=2,
        ))

    # Centre path coloured by IK success
    solved = _solved_xy_set(traj_csv)
    if plan.waypoints:
        xs_p = np.array([w.x for w in plan.waypoints])
        ys_p = np.array([w.y for w in plan.waypoints])
        zs_p = np.array([w.z for w in plan.waypoints])
        if solved:
            success = np.array([(round(x, 3), round(y, 3), round(z, 3)) in solved
                                for x, y, z in zip(xs_p, ys_p, zs_p)])
            for i in range(len(xs_p) - 1):
                col = "#1f9d55" if (success[i] and success[i + 1]) else "#c0392b"
                lw = 1.1 if (success[i] and success[i + 1]) else 0.7
                ax.plot(xs_p[i:i + 2], zs_p[i:i + 2], "-",
                        color=col, linewidth=lw, alpha=0.85, zorder=3)
            n_ok = int(success.sum())
            ax.plot([], [], "-", color="#1f9d55", linewidth=1.5,
                    label=f"Path — IK solved ({n_ok}/{len(success)})")
            ax.plot([], [], "-", color="#c0392b", linewidth=1.5,
                    label=f"Path — IK failed ({len(success) - n_ok}/{len(success)})")
        else:
            ax.plot(xs_p, zs_p, "-", color="#1a3a5c", linewidth=0.9, alpha=0.8,
                    zorder=3, label="Tool centre path")
        ax.plot(xs_p[0], zs_p[0], "o", color="green", markersize=10,
                zorder=4, label=f"Start ({xs_p[0]:+.2f}, {zs_p[0]:+.2f})")
        ax.plot(xs_p[-1], zs_p[-1], "s", color="red", markersize=10,
                zorder=4, label=f"End ({xs_p[-1]:+.2f}, {zs_p[-1]:+.2f})")

    title = (f"Mirror hybrid wipe: horizontal top + vertical sides + roundabout — "
             f"{len(plan.waypoints)} waypoints, "
             f"path {plan.path_length_m:.2f} m, "
             f"{plan.duration_s:.1f} s @ 0.15 m/s")
    if coverage_pct is not None:
        title += f"\nCoverage of reachable mirror area: {coverage_pct:.1f}%"
    ax.set_title(title)
    ax.set_xlabel("X (m, world) — horizontal")
    ax.set_ylabel("Z (m, world) — vertical")
    ax.set_xlim(MIRROR_X[0] - 0.05, MIRROR_X[1] + 0.05)
    ax.set_ylim(MIRROR_Z[0] - 0.05, MIRROR_Z[1] + 0.05)
    ax.set_aspect("equal")
    ax.grid(True, linestyle=":", alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_joint_trajectories(csv_paths: dict, out_path: Path) -> None:
    """csv_paths = {'countertop': Path, 'mirror': Path}"""
    fig, axes = plt.subplots(2, 1, figsize=(12, 9), sharex=False)
    for ax, (surface, csv) in zip(axes, csv_paths.items()):
        if not csv.exists() or csv.stat().st_size < 10:
            ax.set_title(f"{surface}: no IK solutions — nothing to plot")
            ax.set_xlabel("time (s)"); ax.set_ylabel("joint angle (rad)")
            continue
        df = pd.read_csv(csv)
        for joint in [f"joint{i}" for i in range(1, 7)]:
            ax.plot(df["time_s"], df[joint], label=joint, linewidth=1.0)
        ax.set_title(f"{surface}: joint angles vs time "
                      f"({len(df)} points, {df['time_s'].iloc[-1]:.1f} s)")
        ax.set_xlabel("time (s)")
        ax.set_ylabel("joint angle (rad)")
        ax.grid(True, linestyle=":", alpha=0.4)
        ax.legend(loc="upper right", ncol=6, fontsize=8)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_metrics_bar(metrics: dict, out_path: Path) -> None:
    """metrics = {surface: {"coverage_pct": x, "path_length_m": y, "duration_s": z, "waypoints": n}}"""
    fig, axes = plt.subplots(1, 3, figsize=(14, 5))
    surfaces = list(metrics.keys())
    bar_kwargs = dict(width=0.55, color=["#1f9d55", "#3e9fea"])

    axes[0].bar(surfaces, [metrics[s]["coverage_pct"] for s in surfaces], **bar_kwargs)
    axes[0].set_ylabel("Coverage (%)")
    axes[0].set_ylim(0, 105)
    axes[0].set_title("Coverage of reachable patch")
    for i, s in enumerate(surfaces):
        axes[0].text(i, metrics[s]["coverage_pct"] + 1.5,
                     f"{metrics[s]['coverage_pct']:.1f}%", ha="center", fontsize=11)

    axes[1].bar(surfaces, [metrics[s]["path_length_m"] for s in surfaces], **bar_kwargs)
    axes[1].set_ylabel("Path length (m)")
    axes[1].set_title("Cartesian path length")
    for i, s in enumerate(surfaces):
        axes[1].text(i, metrics[s]["path_length_m"] + 0.1,
                     f"{metrics[s]['path_length_m']:.2f} m", ha="center", fontsize=11)

    axes[2].bar(surfaces, [metrics[s]["duration_s"] for s in surfaces], **bar_kwargs)
    axes[2].set_ylabel("Duration (s)")
    axes[2].set_title("Estimated execution time")
    for i, s in enumerate(surfaces):
        axes[2].text(i, metrics[s]["duration_s"] + 1.0,
                     f"{metrics[s]['duration_s']:.1f} s", ha="center", fontsize=11)

    plt.suptitle("Section 2 — coverage strategy metrics", y=1.02, fontsize=13)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
