#!/usr/bin/env python3
"""
animate_demo.py — top-down GIF/MP4 of the naive countertop wipe.

Renders a matplotlib FuncAnimation showing:
  * the countertop outline + faucet keep-out circle
  * the wiped trail (coloured by mode)
  * the live tool tip
  * a side panel with a force gauge and the current mode/time

We deliberately use the *naive* trajectory (which crosses the faucet) so the
arc-over event is the visual highlight. The file lands as
outputs/wiping_demo.mp4 if ffmpeg is available, else outputs/wiping_demo.gif
via Pillow.
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.animation import FuncAnimation, PillowWriter, FFMpegWriter

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from wiping_controller import F_MAX, F_TRIGGER, FAUCET_XY, TARGET_FORCE

OUT_DIR = HERE.parents[1] / "section3" / "outputs"

MODE_COLOR = {
    "approach":       "#f1c40f",
    "force":          "#27ae60",
    "backoff":        "#c0392b",
    "obstacle_avoid": "#8e44ad",
}


def build(log_csv: Path, out_path: Path, surface: str = "countertop",
           target_fps: int = 20, max_seconds: float = 14.0):
    df = pd.read_csv(log_csv)
    # Decimate to target_fps so the file size stays sane.
    dt = df.t.iloc[1] - df.t.iloc[0]
    step = max(1, int(round(1.0 / (target_fps * dt))))
    df = df.iloc[::step].reset_index(drop=True)
    df = df[df.t <= max_seconds].reset_index(drop=True)
    n = len(df)

    target = TARGET_FORCE[surface]

    fig = plt.figure(figsize=(14, 7))
    gs = fig.add_gridspec(1, 3, width_ratios=[3, 0.7, 1.3])
    ax_map = fig.add_subplot(gs[0])
    ax_gauge = fig.add_subplot(gs[1])
    ax_info = fig.add_subplot(gs[2])

    # ── Top-down map ────────────────────────────────────────────────────
    ax_map.add_patch(patches.Rectangle((-0.60, -0.05), 1.20, 0.60,
                                        fill=False, linestyle="--", linewidth=1.5,
                                        edgecolor="black"))
    ax_map.add_patch(patches.Circle(FAUCET_XY, 0.055, fill=True, alpha=0.30,
                                     facecolor="blue", edgecolor="blue",
                                     linewidth=1.5, label="faucet keep-out"))
    ax_map.plot(0, 0.05, "k^", markersize=14, label="arm base")
    ax_map.set_xlim(-0.40, 0.40)
    ax_map.set_ylim(0.35, 0.60)
    ax_map.set_aspect("equal")
    ax_map.set_xlabel("X (m)")
    ax_map.set_ylabel("Y (m)")
    ax_map.set_title("Countertop wipe — naïve trajectory crosses faucet")
    ax_map.grid(True, linestyle=":", alpha=0.3)
    ax_map.legend(loc="lower right", fontsize=8)

    trail = ax_map.scatter([], [], s=18, c=[], cmap="viridis")  # placeholder
    trail_xs, trail_ys, trail_cs = [], [], []
    tool_dot, = ax_map.plot([], [], "o", color="black", markersize=12, zorder=5)

    # ── Force gauge ─────────────────────────────────────────────────────
    ax_gauge.set_title("F_z (N)")
    ax_gauge.set_xticks([])
    ax_gauge.set_ylim(0, 16)
    ax_gauge.axhspan(target - 2, target + 2, color="green", alpha=0.20,
                      label="in-band")
    ax_gauge.axhline(target, color="green", linestyle="--", linewidth=1)
    ax_gauge.axhline(F_TRIGGER, color="orange", linestyle=":", linewidth=1)
    ax_gauge.axhline(F_MAX, color="red", linestyle=":", linewidth=1)
    gauge_bar = ax_gauge.bar([0], [0], width=0.8, color="#2c3e50")
    gauge_text = ax_gauge.text(0, 15.2, "", ha="center", fontsize=11)

    # ── Info panel ──────────────────────────────────────────────────────
    ax_info.axis("off")
    info_text = ax_info.text(0.05, 0.95, "", va="top", ha="left",
                              family="monospace", fontsize=12,
                              transform=ax_info.transAxes)
    legend_handles = [patches.Patch(color=c, label=m) for m, c in MODE_COLOR.items()]
    ax_info.legend(handles=legend_handles, loc="lower left", fontsize=9,
                    bbox_to_anchor=(0.0, 0.0))

    def init():
        return tool_dot, gauge_bar[0], gauge_text, info_text

    def step(i):
        row = df.iloc[i]
        # Append to trail
        trail_xs.append(row.x); trail_ys.append(row.y)
        trail_cs.append(MODE_COLOR.get(row["mode"], "grey"))
        ax_map.scatter(trail_xs, trail_ys, s=14, c=trail_cs, alpha=0.55,
                        edgecolors="none", zorder=2)
        tool_dot.set_data([row.x], [row.y])

        gauge_bar[0].set_height(max(row.force_n, 0.0))
        col = MODE_COLOR.get(row["mode"], "grey")
        gauge_bar[0].set_color(col)
        gauge_text.set_text(f"{row.force_n:.1f} N")

        info_text.set_text(
            f"t       = {row.t:6.2f} s\n"
            f"mode    = {row['mode']:>15s}\n"
            f"x, y    = {row.x:+.3f}, {row.y:+.3f}\n"
            f"z       = {row.z:+.3f} m\n"
            f"F target = {target:5.1f} N\n"
            f"F meas  = {row.force_n:5.2f} N\n"
            f"v_lat   = {row.v_lat_mps:5.3f} m/s\n"
            f"v_norm  = {row.v_norm_cmd_mps:+5.3f} m/s\n"
            f"obstacle = {'YES' if int(row.obstacle_active) else 'no '}"
        )
        return tool_dot, gauge_bar[0], gauge_text, info_text

    anim = FuncAnimation(fig, step, init_func=init, frames=n,
                          interval=1000 / target_fps, blit=False)

    if matplotlib.animation.FFMpegWriter.isAvailable():
        writer = FFMpegWriter(fps=target_fps, bitrate=2000)
        out_file = out_path.with_suffix(".mp4")
        anim.save(out_file, writer=writer, dpi=100)
    else:
        writer = PillowWriter(fps=target_fps)
        out_file = out_path.with_suffix(".gif")
        anim.save(out_file, writer=writer, dpi=65)
    plt.close(fig)
    return out_file


def main():
    log = OUT_DIR / "wipe_log_countertop_naive.csv"
    out = OUT_DIR / "wiping_demo"
    if not log.exists():
        print(f"missing {log} — run run_wiping_demo.py first")
        return
    out_file = build(log, out)
    print(f"  → {out_file.name}")


if __name__ == "__main__":
    main()
