#!/usr/bin/env python3
"""
run_wiping_demo.py — Section 3 simulator + log writer.

Loads Section 2's per-waypoint trajectories (countertop + mirror), then
simulates the wipe at 100 Hz: at each tick the F/T sensor is read, the
controller is updated, and the resulting normal-velocity command is
integrated into the tool tip's z (countertop) or y (mirror). The lateral
motion follows the planned tangent at the per-surface nominal speed,
scaled by the controller (frozen during BACKOFF, slowed during
OBSTACLE_AVOID).

Outputs (per surface):
  outputs/wipe_log_<surface>.csv   columns: t, mode, x, y, z,
                                   force_n, target_n, v_norm_cmd_mps,
                                   v_lat_mps, obstacle_active

The log is what `plot_results.py` and `animate_demo.py` consume.

NB: this is a single-axis admittance sim — for a 2D contact (e.g. tool
also slipping sideways), the same controller runs on the OTHER axis but
the spec only asks for |Fz| tracking, so we keep it 1D.
"""

from __future__ import annotations

import csv
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List

import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from ft_sensor_sim import COUNTERTOP_TOP_Z, MIRROR_FACE_Y, FTSensor
from wiping_controller import (
    NOMINAL_SPEED, TARGET_FORCE, ControllerState, Mode, update,
)

OUT_DIR = HERE.parents[1] / "section3" / "outputs"
OUT_DIR.mkdir(parents=True, exist_ok=True)

S2_OUT = HERE.parents[1] / "section2" / "outputs"
GRIPPER_TIP_OFFSET = 0.1358    # link6 → finger tip (same as Section 1)

DT = 0.01   # 100 Hz sim


# ── Helpers to map link6-frame waypoints to tool-tip-frame Cartesian path ──
def link6_to_tip(surface: str, x: float, y: float, z: float):
    """Convert planner waypoint (link6 pose) to tool-tip world position.

    The planner targets link6; the gripper tip sits GRIPPER_TIP_OFFSET along
    link6's local +Z. With the surface-aligned tool orientation:
      countertop: tool +Z = world -Z  → tip is at (x, y, z - offset)
      mirror:     tool +Z = world +Y  → tip is at (x, y + offset, z)
    """
    if surface == "countertop":
        return np.array([x, y, z - GRIPPER_TIP_OFFSET])
    elif surface == "mirror":
        return np.array([x, y + GRIPPER_TIP_OFFSET, z])
    else:
        raise ValueError(surface)


def tangent_unit(p0: np.ndarray, p1: np.ndarray) -> np.ndarray:
    d = p1 - p0
    n = float(np.linalg.norm(d))
    return d / n if n > 1e-9 else np.zeros_like(d)


def simulate(surface: str, trajectory_csv: Path,
              out_path: Path = None) -> Path:
    df = pd.read_csv(trajectory_csv)
    if len(df) < 2:
        raise RuntimeError(f"{trajectory_csv} has <2 waypoints — nothing to sim")

    # Convert waypoints to tool-tip world positions (only the lateral component matters
    # for path-following; the normal axis is owned by the controller).
    tips = np.array([link6_to_tip(surface, r.x, r.y, r.z) for r in df.itertuples()])

    # Start the tool 3 cm ABOVE the surface so the demo shows APPROACH → contact.
    if surface == "countertop":
        tool = tips[0].copy()
        tool[2] = COUNTERTOP_TOP_Z + 0.03
        normal_axis = 2          # z
        normal_sign = -1         # +normal_velocity (INTO surface) = decrease z
        surface_plane = COUNTERTOP_TOP_Z
    else:  # mirror
        tool = tips[0].copy()
        tool[1] = MIRROR_FACE_Y - 0.03
        normal_axis = 1          # y
        normal_sign = +1         # +normal_velocity (INTO surface) = increase y
        surface_plane = MIRROR_FACE_Y

    sensor = FTSensor()
    state = ControllerState()

    nominal_v = NOMINAL_SPEED[surface]
    target = TARGET_FORCE[surface]

    log_rows: List[dict] = []
    t = 0.0
    wp_idx = 0
    seg_start = tips[wp_idx].copy()
    seg_end = tips[wp_idx + 1].copy()
    seg_progress = 0.0
    seg_len = float(np.linalg.norm(seg_end - seg_start))

    while True:
        # ── Sense ───────────────────────────────────────────────────────
        f = sensor.read(surface, tool)

        # Pick a look-ahead "next_xy" (1 waypoint ahead) for obstacle test.
        look_idx = min(wp_idx + 2, len(tips) - 1)
        next_xy = tips[look_idx][:2]

        # ── Controller ──────────────────────────────────────────────────
        cmd = update(state, surface, f, tool[:2], next_xy, DT)

        # ── Apply normal-axis command ───────────────────────────────────
        dn = normal_sign * cmd.normal_velocity * DT
        if cmd.mode == Mode.OBSTACLE_AVOID:
            # The arc is a setpoint OFFSET, not a velocity — snap to it.
            target_normal = surface_plane + normal_sign * (-cmd.z_offset)
            tool[normal_axis] += 0.4 * (target_normal - tool[normal_axis])  # smooth approach
        else:
            tool[normal_axis] += dn

        # ── Apply lateral motion along the planned tangent ──────────────
        lat_speed = nominal_v * cmd.lateral_scale
        step = lat_speed * DT
        if seg_len > 1e-9:
            tan = tangent_unit(seg_start, seg_end)
            tool[0] += tan[0] * step
            tool[1] += tan[1] * step if surface == "countertop" else 0.0
            if surface == "mirror":
                tool[2] += tan[2] * step
            seg_progress += step

        # ── Log ─────────────────────────────────────────────────────────
        log_rows.append({
            "t":               round(t, 4),
            "mode":            cmd.mode.value,
            "x":               round(float(tool[0]), 5),
            "y":               round(float(tool[1]), 5),
            "z":               round(float(tool[2]), 5),
            "force_n":         round(float(f), 3),
            "target_n":        target,
            "v_norm_cmd_mps":  round(float(cmd.normal_velocity), 4),
            "v_lat_mps":       round(float(lat_speed), 4),
            "obstacle_active": int(state.last_obstacle_active),
        })

        # ── Advance segment ─────────────────────────────────────────────
        if seg_progress >= seg_len:
            wp_idx += 1
            if wp_idx + 1 >= len(tips):
                break
            seg_start = tips[wp_idx].copy()
            # Carry over the controlled normal axis so we don't teleport
            seg_start[normal_axis] = tool[normal_axis]
            seg_end = tips[wp_idx + 1].copy()
            seg_end[normal_axis] = tool[normal_axis]
            seg_progress = 0.0
            seg_len = float(np.linalg.norm(seg_end - seg_start))

        t += DT

        # Safety: hard cap on total sim time so a runaway loop can't hang us.
        if t > 600.0:
            print(f"WARN: {surface} sim hit 600 s cap")
            break

    if out_path is None:
        out_path = OUT_DIR / f"wipe_log_{surface}.csv"
    with out_path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(log_rows[0].keys()))
        w.writeheader()
        w.writerows(log_rows)
    return out_path


def make_naive_obstacle_trajectory() -> Path:
    """Synthesize a naïve raster that runs straight through the faucet zone
    so the controller's OBSTACLE_AVOID arc-over actually fires. Section 2's
    real trajectory already pre-filters the faucet keep-out, so it never
    exercises this code path.

    Two horizontal sweeps at Y = 0.46 and Y = 0.50 (faucet centre is at
    Y = 0.50) spanning X ∈ [-0.30, +0.30] at 2 cm resolution.
    """
    rows = []
    z_target = 0.05 + 0.1358 + 0.005   # link6 Z for tool tip 5 mm above counter
    for y in (0.46, 0.50):
        xs = np.arange(-0.30, 0.30 + 1e-9, 0.02)
        if y == 0.50:
            xs = xs[::-1]
        for x in xs:
            rows.append({"time_s": 0.0,
                          "x": round(float(x), 5),
                          "y": round(float(y), 5),
                          "z": round(float(z_target), 5),
                          "surface": "countertop"})
    df = pd.DataFrame(rows)
    out = OUT_DIR / "trajectory_countertop_naive.csv"
    df.to_csv(out, index=False)
    return out


def main():
    runs = [
        ("countertop",       S2_OUT / "trajectory_countertop.csv"),
        ("mirror",           S2_OUT / "trajectory_mirror.csv"),
    ]

    # Build + add the naive obstacle-demo trajectory
    naive_csv = make_naive_obstacle_trajectory()
    runs.append(("countertop_naive", naive_csv))

    for label, csv_path in runs:
        if not csv_path.exists():
            print(f"SKIP {label}: {csv_path} missing — run Section 2 first")
            continue
        # Real sim still uses the underlying surface name
        surface = "mirror" if label == "mirror" else "countertop"
        print(f"Simulating {label} ({csv_path.name}) …")
        out_csv = OUT_DIR / f"wipe_log_{label}.csv"
        simulate(surface, csv_path, out_path=out_csv)
        df = pd.read_csv(out_csv)
        print(f"  → {out_csv.name}  ({len(df)} samples, {df.t.iloc[-1]:.1f} s wall sim)")
        mode_counts = df["mode"].value_counts()
        print("  mode counts:")
        for m, n in mode_counts.items():
            print(f"    {m:>15}  {n:>6}  ({100 * n / len(df):.1f}%)")
        in_band = (df.force_n.between(
            TARGET_FORCE[surface] - 2.0, TARGET_FORCE[surface] + 2.0)
                   & (df["mode"] == "force")).sum()
        force_steps = (df["mode"] == "force").sum()
        if force_steps:
            print(f"  force-mode in-band:  {in_band}/{force_steps} "
                  f"({100 * in_band / force_steps:.1f}%)")


if __name__ == "__main__":
    main()
