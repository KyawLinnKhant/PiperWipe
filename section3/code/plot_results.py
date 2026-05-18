#!/usr/bin/env python3
"""
plot_results.py — Section 3 static plots.

Produces a 3-panel figure per surface run:

  1.  Force vs time, with target / tolerance / safety bands and mode shading
  2.  Velocity vs time (normal cmd + lateral), with spec speed range band
  3.  Mode timeline (color strip across the time axis)

Plus an obstacle-event close-up for the naive countertop demo.
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from wiping_controller import (
    F_MAX, F_TRIGGER, SPEED_RANGE, TARGET_FORCE, TARGET_TOL,
)

OUT_DIR = HERE.parents[1] / "section3" / "outputs"

MODE_COLOR = {
    "approach":       "#f1c40f",   # amber
    "force":          "#27ae60",   # green
    "backoff":        "#c0392b",   # red
    "obstacle_avoid": "#8e44ad",   # purple
}


def _shade_modes(ax, df, ymin, ymax):
    """Color a thin band across the bottom of `ax` showing the mode timeline."""
    band_h = (ymax - ymin) * 0.04
    last_mode = df["mode"].iloc[0]
    seg_start = df["t"].iloc[0]
    for i in range(1, len(df)):
        if df["mode"].iloc[i] != last_mode:
            ax.axvspan(seg_start, df["t"].iloc[i],
                        ymin=0, ymax=band_h / (ymax - ymin),
                        color=MODE_COLOR.get(last_mode, "grey"), alpha=0.8)
            seg_start = df["t"].iloc[i]
            last_mode = df["mode"].iloc[i]
    ax.axvspan(seg_start, df["t"].iloc[-1],
                ymin=0, ymax=band_h / (ymax - ymin),
                color=MODE_COLOR.get(last_mode, "grey"), alpha=0.8)


def plot_run(csv_path: Path, surface: str, out_path: Path,
              title_suffix: str = "") -> None:
    df = pd.read_csv(csv_path)
    target = TARGET_FORCE[surface]
    tol = TARGET_TOL[surface]
    v_lo, v_hi = SPEED_RANGE[surface]

    fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True,
                             gridspec_kw={"height_ratios": [3, 2, 0.6]})

    # ── 1. Force vs time ────────────────────────────────────────────────
    ax = axes[0]
    ax.plot(df.t, df.force_n, "-", color="#2c3e50", linewidth=1.2,
            label="Measured F_normal")
    ax.axhline(target, color="green", linestyle="--", linewidth=1.0,
                label=f"Target {target:g} N")
    ax.fill_between(df.t, target - tol, target + tol,
                     color="green", alpha=0.10,
                     label=f"In-band ±{tol:g} N")
    ax.axhline(F_TRIGGER, color="orange", linestyle=":", linewidth=0.8,
                label=f"Trigger {F_TRIGGER:g} N")
    ax.axhline(F_MAX, color="red", linestyle=":", linewidth=0.8,
                label=f"Safety cap {F_MAX:g} N")
    ax.set_ylabel("Force (N)")
    ymin, ymax = -1.0, max(F_MAX + 2.0, float(df.force_n.max()) + 1.0)
    ax.set_ylim(ymin, ymax)
    _shade_modes(ax, df, ymin, ymax)
    ax.grid(True, linestyle=":", alpha=0.4)
    ax.legend(loc="upper right", ncol=3, fontsize=8)

    # ── 2. Velocity vs time ─────────────────────────────────────────────
    ax = axes[1]
    ax.plot(df.t, df.v_lat_mps, "-", color="#1f3d5c", linewidth=1.2,
            label="Lateral speed (commanded)")
    ax.plot(df.t, df.v_norm_cmd_mps.abs(), "-", color="#c0392b",
            linewidth=1.0, alpha=0.75,
            label="Normal speed cmd (|·|)")
    ax.fill_between(df.t, v_lo, v_hi, color="#1f3d5c", alpha=0.10,
                     label=f"Spec band {v_lo:g}-{v_hi:g} m/s")
    ax.set_ylabel("Velocity (m/s)")
    ymin, ymax = -0.005, max(v_hi + 0.02, float(df.v_lat_mps.max()) + 0.02)
    ax.set_ylim(ymin, ymax)
    _shade_modes(ax, df, ymin, ymax)
    ax.grid(True, linestyle=":", alpha=0.4)
    ax.legend(loc="upper right", ncol=3, fontsize=8)

    # ── 3. Mode timeline ─────────────────────────────────────────────────
    ax = axes[2]
    ax.set_yticks([])
    ax.set_xlabel("time (s)")
    last_mode = df["mode"].iloc[0]
    seg_start = df["t"].iloc[0]
    for i in range(1, len(df)):
        if df["mode"].iloc[i] != last_mode:
            ax.axvspan(seg_start, df["t"].iloc[i],
                        color=MODE_COLOR.get(last_mode, "grey"), alpha=0.85)
            seg_start = df["t"].iloc[i]
            last_mode = df["mode"].iloc[i]
    ax.axvspan(seg_start, df["t"].iloc[-1],
                color=MODE_COLOR.get(last_mode, "grey"), alpha=0.85)
    # Legend
    handles = [patches.Patch(color=c, label=m) for m, c in MODE_COLOR.items()]
    ax.legend(handles=handles, loc="upper right", ncol=4, fontsize=8,
              frameon=True)
    ax.set_xlim(df.t.iloc[0], df.t.iloc[-1])

    # Summary stats
    in_band = ((df["mode"] == "force") &
               df.force_n.between(target - tol, target + tol)).sum()
    force_steps = (df["mode"] == "force").sum()
    pct = 100 * in_band / force_steps if force_steps else 0.0
    avg = float(df.loc[df["mode"] == "force", "force_n"].mean()) if force_steps else 0.0

    plt.suptitle(
        f"{surface.title()} wipe — Section 3 controller{title_suffix}\n"
        f"force-mode samples: {force_steps},  "
        f"in-band: {in_band}/{force_steps} ({pct:.1f}%),  "
        f"mean F_force = {avg:.2f} N (target {target:g} N ±{tol:g})",
        y=0.995, fontsize=12,
    )
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_obstacle_event(csv_path: Path, out_path: Path) -> None:
    """Close-up on the first obstacle-avoid event in the naive run."""
    df = pd.read_csv(csv_path)
    avoid = df[df["mode"] == "obstacle_avoid"]
    if avoid.empty:
        return
    t0 = avoid.t.iloc[0]
    window = df[(df.t >= t0 - 1.0) & (df.t <= avoid.t.iloc[-1] + 2.0)].copy()

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True,
                             gridspec_kw={"height_ratios": [2, 2, 1]})

    ax = axes[0]
    ax.plot(window.t, window.force_n, "-", color="#2c3e50", linewidth=1.4,
             label="F_normal")
    ax.axhline(TARGET_FORCE["countertop"], color="green", linestyle="--",
                linewidth=0.8, label="Target 10 N")
    ax.axhline(F_TRIGGER, color="orange", linestyle=":", linewidth=0.8,
                label="Trigger 2 N")
    ax.set_ylabel("Force (N)"); ax.grid(True, linestyle=":", alpha=0.4)
    ax.legend(loc="upper right", fontsize=8)

    ax = axes[1]
    ax.plot(window.t, (window.z - 0.05) * 1000, "-", color="#1f3d5c",
             linewidth=1.4, label="Tool tip Z above counter (mm)")
    ax.axhline(0, color="black", linestyle=":", linewidth=0.6,
                label="Counter surface")
    ax.set_ylabel("Z height (mm)"); ax.grid(True, linestyle=":", alpha=0.4)
    ax.legend(loc="upper right", fontsize=8)

    ax = axes[2]; ax.set_yticks([])
    last_mode = window["mode"].iloc[0]
    seg_start = window["t"].iloc[0]
    for i in range(1, len(window)):
        if window["mode"].iloc[i] != last_mode:
            ax.axvspan(seg_start, window["t"].iloc[i],
                        color=MODE_COLOR.get(last_mode, "grey"), alpha=0.85)
            seg_start = window["t"].iloc[i]
            last_mode = window["mode"].iloc[i]
    ax.axvspan(seg_start, window["t"].iloc[-1],
                color=MODE_COLOR.get(last_mode, "grey"), alpha=0.85)
    ax.set_xlabel("time (s)")
    handles = [patches.Patch(color=c, label=m) for m, c in MODE_COLOR.items()]
    ax.legend(handles=handles, loc="upper right", ncol=4, fontsize=8)
    ax.set_xlim(window.t.iloc[0], window.t.iloc[-1])

    plt.suptitle("Obstacle (faucet) event — controller arcs over and re-acquires contact",
                  y=1.0, fontsize=12)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close(fig)


def main():
    runs = [
        ("countertop",       "countertop",                   ""),
        ("mirror",           "mirror",                       ""),
        ("countertop_naive", "countertop",
            "  —  naïve trajectory crosses faucet (demo)"),
    ]
    for label, surface, suffix in runs:
        csv = OUT_DIR / f"wipe_log_{label}.csv"
        if not csv.exists():
            print(f"SKIP {label}: {csv} missing")
            continue
        out = OUT_DIR / f"{label}_tracking.png"
        plot_run(csv, surface, out, title_suffix=suffix)
        print(f"  → {out.name}")

    naive_csv = OUT_DIR / "wipe_log_countertop_naive.csv"
    if naive_csv.exists():
        out = OUT_DIR / "obstacle_event_closeup.png"
        plot_obstacle_event(naive_csv, out)
        print(f"  → {out.name}")


if __name__ == "__main__":
    main()
