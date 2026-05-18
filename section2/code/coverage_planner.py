"""
coverage_planner.py — Section 2 path generation.

Generates Cartesian waypoint paths for two coverage strategies:

  * raster_countertop()  – boustrophedon (snake) sweep over the countertop.
    Pad long axis (100 mm) is laid perpendicular to the sweep direction so
    each pass covers a 100 mm Y-strip; rows are spaced by
    100 mm × (1 − overlap) for 10-20 % overlap. Inter-row transitions that
    would drag the arm through the faucet keep-out get an explicit
    lift+lateral+drop arc inserted.

  * semicircle_mirror() – concentric semicircular wipe arcs centred at the
    bottom-middle of the mirror (the "windshield wiper" pattern). Each pass
    is a half-circle at a fixed radius; successive arcs are spaced by the
    overlap-corrected pad pitch. Naturally fits the reachable DOME shape
    better than horizontal stripes — covers more of the wide cells at the
    arc's outer reach and stays clear of the bottom-centre exclusion.

  * raster_mirror()      – (kept as fallback) top-down boustrophedon sweep
    over the same area; horizontal stripes step DOWN by pad pitch.

Constraints (from the assignment):
    - Tool pad footprint: 100 × 50 mm
    - Overlap: 10–20 %  (default 15 %)
    - Keep-out margin from any edge / obstacle: 15 mm
    - Tool normal ≈ surface normal ± 10°

Both planners consult the appropriate Section 1 reachability CSV and drop
waypoints in unreachable cells before IK is even attempted.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import pandas as pd


# ── Geometry / tool / scene constants ────────────────────────────────────────
PAD_LONG = 0.100        # m, perpendicular to sweep direction (swath width)
PAD_SHORT = 0.050       # m, along sweep direction
OVERLAP = 0.15          # 15 %, inside the 10-20 % spec band
KEEPOUT = 0.015         # m

# Countertop (from kitchen_scene.py): centre (0, 0.25, 0.025), 1.20 × 0.60 × 0.05.
COUNTERTOP_X = (-0.60, 0.60)
COUNTERTOP_Y = (-0.05, 0.55)
COUNTERTOP_TOP_Z = 0.05

# Mirror: centre (0, 0.555, 0.50), 0.60 × 0.01 × 0.90.
MIRROR_FACE_Y = 0.555 - 0.005   # front face of the 1-cm-thick mirror panel
MIRROR_X = (-0.30, 0.30)
MIRROR_Z = (0.05, 0.95)

# Faucet keep-out (cylinder at (0, 0.50), r=15 mm) — inflate by 15 mm margin
# plus pad half-width so the pad footprint never enters the obstacle.
FAUCET_XY = (0.00, 0.50)
# Empirically widened to 10 cm radius after RViz replay showed the arm body
# (not just the pad) crossing the faucet cylinder when wiping cells just
# outside the original 5.5 cm pad-clearance disk.
FAUCET_KEEPOUT_R = 0.10     # m — generous 10 cm bubble around faucet centre
FAUCET_ARC_HEIGHT = 0.08    # m — link6 lift above counter when arcing over faucet
# In addition to the disk, ban the 5 cm-wide X-strip down the middle of the
# countertop for any cell BEHIND the front half (Y > 0.30), i.e. the column
# the faucet stands in. Stops the arm from threading the gripper into that
# column when reaching for back cells next to the faucet.
FAUCET_COLUMN_X_HALF = 0.025   # ±2.5 cm strip
FAUCET_COLUMN_Y_MIN = 0.30     # only the back half of counter (faucet at y=0.50)

# Mirror — empirical planner constraints discovered during RViz replay:
#   * cells above Z = MIRROR_MAX_WIPE_Z have very few IK solutions and make
#     the arm spin haywire at the topmost ring; cap the raster below it.
#   * the bottom-centre of the mirror is right above the faucet column, so
#     the arm body fouls the faucet when wiping there — carve out a 10 cm
#     radius SEMICIRCLE centred on the bottom edge (X=0, Z=0).
MIRROR_MAX_WIPE_Z = 0.45              # m — cap a bit below the topmost reach
MIRROR_BOTTOM_BAN_RADIUS = 0.13       # m — semicircle radius at bottom-middle (enlarged for gripper-body clearance)

# Gripper-tip offset along link6's +Z (see Section 1 writeup).
GRIPPER_TIP_OFFSET = 0.1358
TIP_CLEARANCE = 0.005
COUNTER_Z_TARGET = COUNTERTOP_TOP_Z + GRIPPER_TIP_OFFSET + TIP_CLEARANCE  # 0.1908 m
MIRROR_Y_TARGET = MIRROR_FACE_Y - (GRIPPER_TIP_OFFSET + TIP_CLEARANCE)    # ≈ 0.4102 m

SECTION1_CSV = Path(__file__).resolve().parents[2] / "section1" / "outputs" / "reachability.csv"


@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    surface: str       # "countertop" or "mirror"
    yaw: float = 0.0   # TCP yaw (rad) about the tool approach axis
    pitch: float = 0.0 # tilt about tool +X axis (rad)
    # Optional per-waypoint preferred IK orientation. When set, the trajectory
    # builder tries this (pitch_deg, yaw_deg) FIRST before falling back to the
    # surface's default lattice. Used to rotate the sponge between "vertical"
    # (perpendicular to motion → wide swath) during a sweep and "horizontal"
    # (parallel to motion direction) during a transition.
    preferred_pitch_deg: Optional[float] = None
    preferred_yaw_deg: Optional[float] = None


@dataclass
class CoveragePlan:
    surface: str
    waypoints: List[Waypoint]
    # Each entry = (x, y, z) corners of the pad footprint, world frame — used
    # by the visualizer to draw the wiped swath.
    swath_polys: List[np.ndarray] = field(default_factory=list)
    # Patch dimensions (for coverage-% normalization).
    patch_extent: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    # m, path arc-length and timing populated by trajectory_builder.
    path_length_m: float = 0.0
    duration_s: float = 0.0


# ── Helpers ──────────────────────────────────────────────────────────────────

def _load_reachable_mask(csv_path: Path = SECTION1_CSV):
    """Return (xs, ys, mask) where mask[iy, ix] is True for reachable cells."""
    df = pd.read_csv(csv_path)
    xs = np.array(sorted(df["x"].unique()))
    ys = np.array(sorted(df["y"].unique()))
    pivot = (
        df.pivot(index="y", columns="x", values="reachable")
        .reindex(index=ys, columns=xs)
        .fillna(0)
        .astype(int)
        .to_numpy()
    )
    return xs, ys, pivot.astype(bool)


def _nearest_reachable(x: float, y: float, xs, ys, mask) -> bool:
    """Look up reachability for the nearest 2 cm cell."""
    ix = int(np.argmin(np.abs(xs - x)))
    iy = int(np.argmin(np.abs(ys - y)))
    return bool(mask[iy, ix])


def _faucet_clear(x: float, y: float) -> bool:
    """True if cell is safely outside BOTH the 10 cm faucet disk AND the
    5 cm centre-column strip behind Y = 0.30."""
    dx, dy = x - FAUCET_XY[0], y - FAUCET_XY[1]
    if (dx * dx + dy * dy) < FAUCET_KEEPOUT_R ** 2:
        return False
    if abs(x) < FAUCET_COLUMN_X_HALF and y > FAUCET_COLUMN_Y_MIN:
        return False
    return True


def _mirror_bottom_clear(x: float, z: float) -> bool:
    """True if the mirror cell is OUTSIDE the 10 cm-radius semicircle at the
    bottom-middle of the mirror (centred at X=0, Z=0). The semicircle bulges
    UP into the mirror so it carves out everything within 10 cm of the
    bottom-centre point — right above where the faucet stands."""
    if z < 0:
        return True
    return (x * x + z * z) >= MIRROR_BOTTOM_BAN_RADIUS ** 2


def _segment_crosses_faucet(p0: tuple, p1: tuple) -> bool:
    """True if the straight line p0→p1 (in XY) passes through the faucet
    keep-out disk. Used so the raster can route an in-plane roundabout
    around the faucet instead of letting the arm sweep through it."""
    fx, fy = FAUCET_XY
    x0, y0 = p0; x1, y1 = p1
    dx, dy = x1 - x0, y1 - y0
    seg2 = dx * dx + dy * dy
    if seg2 < 1e-9:
        return _faucet_clear(x0, y0) is False
    # parameter t of foot of perpendicular from faucet centre to segment
    t = ((fx - x0) * dx + (fy - y0) * dy) / seg2
    t = max(0.0, min(1.0, t))
    cx = x0 + t * dx
    cy = y0 + t * dy
    return ((cx - fx) ** 2 + (cy - fy) ** 2) < FAUCET_KEEPOUT_R ** 2


def _roundabout_around_faucet(p0: tuple, p1: tuple,
                               n_samples: int = 12) -> List[Tuple[float, float]]:
    """Generate intermediate (x, y) waypoints that arc AROUND the faucet
    from p0 to p1, staying on the surface at radius FAUCET_KEEPOUT_R.

    Picks the shorter angular direction. Returns the in-between samples
    (does NOT include p0 or p1)."""
    fx, fy = FAUCET_XY
    r = FAUCET_KEEPOUT_R + 0.002   # 2 mm safety inside the keep-out disk
    a0 = math.atan2(p0[1] - fy, p0[0] - fx)
    a1 = math.atan2(p1[1] - fy, p1[0] - fx)
    sweep = a1 - a0
    while sweep >  math.pi: sweep -= 2 * math.pi
    while sweep < -math.pi: sweep += 2 * math.pi
    pts = []
    for i in range(1, n_samples):
        t = i / n_samples
        theta = a0 + t * sweep
        pts.append((fx + r * math.cos(theta), fy + r * math.sin(theta)))
    return pts


def _pad_polygon(x: float, y: float, long_axis: str = "y") -> np.ndarray:
    """Return the 4 corners of the pad footprint centred at (x, y).
    long_axis = 'y' means PAD_LONG runs along Y (default for raster sweep along X).
    """
    half_l = PAD_LONG / 2
    half_s = PAD_SHORT / 2
    if long_axis == "y":
        dx, dy = half_s, half_l
    else:
        dx, dy = half_l, half_s
    return np.array([
        [x - dx, y - dy],
        [x + dx, y - dy],
        [x + dx, y + dy],
        [x - dx, y + dy],
    ])


# ── Raster: countertop ───────────────────────────────────────────────────────

def raster_countertop(
    overlap: float = OVERLAP,
    sample_step: float = 0.02,         # 2 cm Cartesian step along each pass
    use_reachable_mask: bool = True,
) -> CoveragePlan:
    """Boustrophedon raster over the countertop.

    Pad's 100 mm long edge runs along Y (perpendicular to sweep) so each pass
    paints a 100 mm-tall Y strip; rows are spaced by 100 mm × (1 − overlap).
    """
    # Plan envelope = countertop minus the 15 mm edge keep-out, minus the
    # half-pad so the FOOTPRINT (not centre) clears every edge.
    x_min = COUNTERTOP_X[0] + KEEPOUT + PAD_SHORT / 2
    x_max = COUNTERTOP_X[1] - KEEPOUT - PAD_SHORT / 2
    y_min = COUNTERTOP_Y[0] + KEEPOUT + PAD_LONG / 2
    y_max = COUNTERTOP_Y[1] - KEEPOUT - PAD_LONG / 2

    row_pitch = PAD_LONG * (1 - overlap)   # 85 mm at 15 % overlap
    # Snake from the BACK of the counter (nearest the mirror / faucet) toward
    # the FRONT (nearest the arm base). Matches the mirror's top-down
    # direction so the combined demo flows mirror→countertop without flipping.
    y_rows = np.arange(y_min, y_max + 1e-9, row_pitch)[::-1]

    xs, ys_mask, reach_mask = _load_reachable_mask() if use_reachable_mask else (None,) * 3

    waypoints: List[Waypoint] = []
    swath_polys: List[np.ndarray] = []
    direction = +1
    z_target = COUNTER_Z_TARGET
    z_arc = COUNTER_Z_TARGET + FAUCET_ARC_HEIGHT
    # Sponge orientation policy:
    #   SWEEP (motion along X within a row)   → tcp_yaw = 90°  (long axis ⟂ motion, wide swath)
    #   TURN  (motion along Y between rows)   → tcp_yaw =  0°  (long axis ⟂ Y-motion)
    YAW_SWEEP_DEG = 90.0
    YAW_TURN_DEG  =  0.0
    prev_wp = None

    def _maybe_insert_arc(prev: Waypoint, x: float, y: float):
        """In-plane roundabout: if the straight segment prev→(x,y) crosses
        the faucet keep-out disk, insert intermediate waypoints that arc
        around the disk at radius FAUCET_KEEPOUT_R. Tool stays on the
        surface throughout. Roundabout uses TURN orientation."""
        if prev is None:
            return
        if not _segment_crosses_faucet((prev.x, prev.y), (x, y)):
            return
        for (ax, ay) in _roundabout_around_faucet((prev.x, prev.y), (x, y)):
            waypoints.append(Waypoint(ax, ay, z_target, surface="countertop",
                                       preferred_yaw_deg=YAW_TURN_DEG))

    for y in y_rows:
        line_xs = np.arange(x_min, x_max + 1e-9, sample_step)
        if direction < 0:
            line_xs = line_xs[::-1]
        first_x_in_row = None
        for x in line_xs:
            in_keepout = not _faucet_clear(x, y)
            if use_reachable_mask and not _nearest_reachable(x, y, xs, ys_mask, reach_mask):
                continue
            if in_keepout:
                continue   # never wipe cells under the faucet itself

            # Inter-row TRANSITION: rotate sponge to TURN orientation AT the
            # last sweep waypoint, traverse to the new row's first X at TURN
            # orientation, then resume SWEEP at the new wipe waypoint.
            if first_x_in_row is None and prev_wp is not None:
                waypoints.append(Waypoint(prev_wp.x, prev_wp.y, z_target,
                                          surface="countertop",
                                          preferred_yaw_deg=YAW_TURN_DEG))
                waypoints.append(Waypoint(float(x), float(y), z_target,
                                          surface="countertop",
                                          preferred_yaw_deg=YAW_TURN_DEG))

            _maybe_insert_arc(prev_wp, float(x), float(y))
            wp = Waypoint(x=float(x), y=float(y), z=z_target,
                          surface="countertop",
                          preferred_yaw_deg=YAW_SWEEP_DEG)
            waypoints.append(wp)
            prev_wp = wp
            if first_x_in_row is None:
                first_x_in_row = float(x)
        direction *= -1

    # Build swath polygons (one per surface-contact waypoint) for visualization.
    for wp in waypoints:
        # Swaths only count when the tool is on the surface, not while arcing.
        if abs(wp.z - z_target) > 1e-4:
            continue
        swath_polys.append(_pad_polygon(wp.x, wp.y, long_axis="y"))

    return CoveragePlan(
        surface="countertop",
        waypoints=waypoints,
        swath_polys=swath_polys,
        patch_extent=(x_min - PAD_SHORT / 2, x_max + PAD_SHORT / 2,
                      y_min - PAD_LONG / 2, y_max + PAD_LONG / 2),
    )


# ── Mirror: hybrid (horizontal top + vertical sides + faucet roundabout) ────

def mirror_wipe(
    overlap: float = OVERLAP,
    sample_step: float = 0.02,
    use_reachable_mask: bool = True,
) -> CoveragePlan:
    """Three-phase mirror wipe that respects the faucet column at the bottom.

    The 10 cm-radius bottom-centre semicircle exclusion splits the lower
    rows of the reachable dome into a LEFT half and a RIGHT half. A simple
    horizontal raster either picks one side and skips the other, or weaves
    through the gap with awkward inter-row diagonals. The hybrid pattern:

    Phase 1 — TOP horizontal raster, snake.
        Rows from z_top down to z_split, sweeping the full reachable X
        width at each Z. Same as a normal top-down raster, but stops just
        ABOVE the faucet column so we never have to bridge the gap.

    Phase 2 — RIGHT vertical strips, top→bottom.
        For each X column to the right of the faucet (starting from the
        outermost and working inward), sweep Z from z_split down to z_min.
        The pad's 100 mm long axis is now along X (perpendicular to motion)
        so each vertical pass paints a 100 mm-wide column.

    Phase 3 — Faucet ROUNDABOUT.
        Arc the tool from the inner-bottom of the right strip to the
        inner-bottom of the left strip, riding the outside of the faucet
        semicircle (radius = MIRROR_BOTTOM_BAN_RADIUS + half-pad). Tool
        stays IN CONTACT with the mirror — no lifting.

    Phase 4 — LEFT vertical strips, bottom→top.
        For each X column to the left of the faucet (starting from the
        innermost and working outward), sweep Z from z_min back up to
        z_split.
    """
    x_left_outer  = MIRROR_X[0] + KEEPOUT + PAD_SHORT / 2
    x_right_outer = MIRROR_X[1] - KEEPOUT - PAD_SHORT / 2
    z_top  = min(MIRROR_Z[1] - KEEPOUT - PAD_LONG / 2, MIRROR_MAX_WIPE_Z)
    z_min  = MIRROR_Z[0] + KEEPOUT + PAD_LONG / 2          # 0.115 m

    # Z above which we do horizontal raster; below which we do vertical strips.
    # Sits just above where the bottom-centre semicircle starts to bite
    # (semicircle radius 0.10, so z=0.20 is comfortably above its top).
    z_split = MIRROR_BOTTOM_BAN_RADIUS + PAD_LONG          # 0.20 m

    row_pitch = PAD_LONG * (1 - overlap)                    # 85 mm (horizontal phase)
    # Vertical-strip phase: sponge stays VERTICAL (long axis along world Z),
    # so motion-direction == long-axis direction during the Z-sweep. Swath
    # is then PAD_SHORT (50 mm) wide → pitch in X must be PAD_SHORT × overlap.
    vstrip_pitch = PAD_SHORT * (1 - overlap)                # 42.5 mm
    # Inside edge of each vertical strip — just outside the bottom-centre
    # semicircle radius, plus a half-pad so the sponge clears the exclusion.
    x_inner = MIRROR_BOTTOM_BAN_RADIUS + PAD_SHORT / 2 + 0.005   # 0.16 m

    # Load reachability mask
    reach_xs = reach_zs = None
    reach_mask = None
    if use_reachable_mask:
        mirror_csv = Path(__file__).resolve().parents[2] / \
            "section1" / "outputs" / "reachability_mirror.csv"
        if not mirror_csv.exists():
            raise FileNotFoundError(
                f"{mirror_csv} missing — run reachability_heatmap_mirror.py first"
            )
        df = pd.read_csv(mirror_csv)
        reach_xs = np.array(sorted(df["x"].unique()))
        reach_zs = np.array(sorted(df["z"].unique()))
        reach_mask = (
            df.pivot(index="z", columns="x", values="reachable")
              .reindex(index=reach_zs, columns=reach_xs)
              .fillna(0).astype(int).to_numpy().astype(bool)
        )

    def _is_reachable(x: float, z: float) -> bool:
        if not _mirror_bottom_clear(x, z):
            return False
        if x < x_left_outer or x > x_right_outer or z < 0 or z > z_top + 1e-9:
            return False
        if reach_mask is None:
            return True
        ix = int(np.argmin(np.abs(reach_xs - x)))
        iz = int(np.argmin(np.abs(reach_zs - z)))
        return bool(reach_mask[iz, ix])

    waypoints: List[Waypoint] = []
    swath_polys: List[np.ndarray] = []

    # ── Phase 1: horizontal raster, top → just above faucet ──────────────
    z_rows = list(np.arange(z_top, z_split - 1e-9, -row_pitch))
    direction = +1
    for z in z_rows:
        sample_xs = np.arange(x_left_outer, x_right_outer + 1e-9, sample_step)
        if direction < 0:
            sample_xs = sample_xs[::-1]
        for x in sample_xs:
            if not _is_reachable(float(x), float(z)):
                continue
            wp = Waypoint(x=float(x), y=MIRROR_Y_TARGET, z=float(z),
                          surface="mirror")
            waypoints.append(wp)
            swath_polys.append(_pad_polygon(float(x), float(z), long_axis="y"))
        direction *= -1

    # ── Phase 2: right vertical strips, top → bottom ─────────────────────
    # Column X values from outermost RIGHT to innermost (= x_inner).
    x_right_cols = list(np.arange(x_right_outer, x_inner - 1e-9, -vstrip_pitch))
    direction_z = -1   # first vertical pass goes DOWN
    for x in x_right_cols:
        z_samples = np.arange(z_split, z_min - 1e-9, -sample_step)
        if direction_z > 0:
            z_samples = z_samples[::-1]
        for z in z_samples:
            if not _is_reachable(float(x), float(z)):
                continue
            wp = Waypoint(x=float(x), y=MIRROR_Y_TARGET, z=float(z),
                          surface="mirror")
            waypoints.append(wp)
            # Sponge is VERTICAL (long axis along world Z) regardless of leg —
            # consistent visualization that matches the IK yaw preference.
            swath_polys.append(_pad_polygon(float(x), float(z), long_axis="y"))
        direction_z *= -1

    # ── Phase 3: roundabout around faucet semicircle ─────────────────────
    # Arc radius just outside the bottom-centre exclusion. Sweep CCW (over
    # the top of the semicircle) from inner-right to inner-left at z_min.
    arc_r = MIRROR_BOTTOM_BAN_RADIUS + PAD_LONG / 2 + 0.005   # 0.155 m
    # Start angle = position of (x_inner, z_min) relative to (0, 0).
    a0 = math.atan2(z_min, x_inner)
    a1 = math.atan2(z_min, -x_inner)
    n_arc = max(8, int(arc_r * abs(a1 - a0) / sample_step))
    for i in range(1, n_arc):   # exclude endpoints — strip waypoints cover them
        t = i / n_arc
        theta = a0 + t * (a1 - a0)
        x = arc_r * math.cos(theta)
        z = arc_r * math.sin(theta)
        if not _is_reachable(float(x), float(z)):
            continue
        wp = Waypoint(x=float(x), y=MIRROR_Y_TARGET, z=float(z),
                      surface="mirror")
        waypoints.append(wp)
        swath_polys.append(_pad_polygon(float(x), float(z), long_axis="y"))

    # ── Phase 4: left vertical strips, bottom → top (continuing the sweep) ─
    x_left_cols = list(np.arange(-x_inner, x_left_outer - 1e-9, -vstrip_pitch))
    direction_z = +1   # coming out of the roundabout heading UP
    for x in x_left_cols:
        z_samples = np.arange(z_min, z_split + 1e-9, sample_step)
        if direction_z < 0:
            z_samples = z_samples[::-1]
        for z in z_samples:
            if not _is_reachable(float(x), float(z)):
                continue
            wp = Waypoint(x=float(x), y=MIRROR_Y_TARGET, z=float(z),
                          surface="mirror")
            waypoints.append(wp)
            # Sponge is VERTICAL (long axis along world Z) regardless of leg —
            # consistent visualization that matches the IK yaw preference.
            swath_polys.append(_pad_polygon(float(x), float(z), long_axis="y"))
        direction_z *= -1

    return CoveragePlan(
        surface="mirror",
        waypoints=waypoints,
        swath_polys=swath_polys,
        patch_extent=(x_left_outer - PAD_SHORT / 2,
                      x_right_outer + PAD_SHORT / 2,
                      0.0, z_top + PAD_LONG / 2),
    )


# Backwards-compat: old code might still import `semicircle_mirror`.
semicircle_mirror = mirror_wipe


def raster_mirror(
    overlap: float = OVERLAP,
    sample_step: float = 0.02,
    use_reachable_mask: bool = True,
) -> CoveragePlan:
    """Top-down boustrophedon raster over the mirror's REACHABLE region.

    Each horizontal pass sweeps left↔right at a fixed Z, the pad's 100 mm
    long axis runs along Z so each pass paints a 100 mm-tall Z-strip.  Rows
    step DOWN from the top of the reachable area by PAD_LONG × (1 − overlap).
    Within each row we clip the X span to the cells that the mirror
    reachability mask marks reachable — so the top rows are NARROW (only
    the centre of the dome) and the lower rows widen out to full mirror
    width. Mirrors how a person wipes a mirror with downward strokes.
    """
    x_min_full = MIRROR_X[0] + KEEPOUT + PAD_SHORT / 2
    x_max_full = MIRROR_X[1] - KEEPOUT - PAD_SHORT / 2
    z_min = MIRROR_Z[0] + KEEPOUT + PAD_LONG / 2
    # Hard cap below the topmost reachable ring — the cells above MIRROR_MAX_WIPE_Z
    # have so few admissible IK branches that the arm wrist flips wildly.
    z_max = min(MIRROR_Z[1] - KEEPOUT - PAD_LONG / 2, MIRROR_MAX_WIPE_Z)

    pitch = PAD_LONG * (1 - overlap)   # 85 mm row step

    # Load mirror reachability mask (required for clipping)
    if not use_reachable_mask:
        # Fall back to the full mirror rectangle.
        reach_xs = reach_zs = None
        reach_mask = None
    else:
        mirror_csv = Path(__file__).resolve().parents[2] / \
            "section1" / "outputs" / "reachability_mirror.csv"
        if not mirror_csv.exists():
            raise FileNotFoundError(
                f"{mirror_csv} missing — run reachability_heatmap_mirror.py first"
            )
        df = pd.read_csv(mirror_csv)
        reach_xs = np.array(sorted(df["x"].unique()))
        reach_zs = np.array(sorted(df["z"].unique()))
        reach_mask = (
            df.pivot(index="z", columns="x", values="reachable")
              .reindex(index=reach_zs, columns=reach_xs)
              .fillna(0).astype(int).to_numpy().astype(bool)
        )

    def _is_reachable(x: float, z: float) -> bool:
        # Hard exclude the bottom-centre box first (faucet column proximity)
        if not _mirror_bottom_clear(x, z):
            return False
        if reach_mask is None:
            return True
        ix = int(np.argmin(np.abs(reach_xs - x)))
        iz = int(np.argmin(np.abs(reach_zs - z)))
        return bool(reach_mask[iz, ix])

    waypoints: List[Waypoint] = []
    leg_axis_per_wp: List[str] = []

    # Stripe top-down. For each Z, scan X across the row and keep only the
    # contiguous segment of reachable X values (clipping the row to the dome).
    z_rows = np.arange(z_max, z_min - 1e-9, -pitch)
    direction = +1
    for z in z_rows:
        # Find the leftmost and rightmost reachable X at this Z, snapped to
        # sample_step grid.
        candidate_xs = np.arange(x_min_full, x_max_full + 1e-9, sample_step)
        mask = np.array([_is_reachable(float(x), float(z)) for x in candidate_xs])
        if not mask.any():
            continue
        # Take the longest run of consecutive True so we don't bridge a
        # split (e.g. two separated reachable bands around an obstacle).
        runs = []
        i = 0
        while i < len(mask):
            if mask[i]:
                j = i
                while j < len(mask) and mask[j]:
                    j += 1
                runs.append((i, j))
                i = j
            else:
                i += 1
        if not runs:
            continue
        # pick the widest run
        a, b = max(runs, key=lambda r: r[1] - r[0])
        row_xs = candidate_xs[a:b]
        if direction < 0:
            row_xs = row_xs[::-1]
        for x in row_xs:
            wp = Waypoint(x=float(x), y=MIRROR_Y_TARGET, z=float(z),
                          surface="mirror")
            waypoints.append(wp)
            leg_axis_per_wp.append("h")
        direction *= -1

    # Pad swaths: horizontal sweep → long axis along Z (drawn as polygon
    # in (x, z) plane with long_axis="y" in 2-D polygon coords).
    swath_polys: List[np.ndarray] = []
    for wp, axis in zip(waypoints, leg_axis_per_wp):
        long = "y"   # always horizontal sweep in this version
        swath_polys.append(_pad_polygon(wp.x, wp.z, long_axis=long))

    return CoveragePlan(
        surface="mirror",
        waypoints=waypoints,
        swath_polys=swath_polys,
        patch_extent=(x_min_full - PAD_SHORT / 2, x_max_full + PAD_SHORT / 2,
                      z_min - PAD_LONG / 2, z_max + PAD_LONG / 2),
    )




# ── Metrics ──────────────────────────────────────────────────────────────────

def path_length(plan: CoveragePlan) -> float:
    """Total Cartesian arc-length of the centre-trajectory (m)."""
    if len(plan.waypoints) < 2:
        return 0.0
    pts = np.array(
        [(w.x, w.y, w.z) for w in plan.waypoints],
        dtype=float,
    )
    deltas = np.diff(pts, axis=0)
    return float(np.linalg.norm(deltas, axis=1).sum())


def coverage_fraction(plan: CoveragePlan, grid_res: float = 0.005) -> float:
    """Fraction of the patch area covered by the union of pad footprints.

    Rasterises each axis-aligned pad rectangle into a grid_res grid over the
    patch_extent and counts unique covered cells. Simple, exact for AA boxes.
    """
    x0, x1, y0, y1 = plan.patch_extent
    nx = max(1, int(np.ceil((x1 - x0) / grid_res)))
    ny = max(1, int(np.ceil((y1 - y0) / grid_res)))
    grid = np.zeros((ny, nx), dtype=bool)
    for poly in plan.swath_polys:
        px_min, py_min = poly.min(axis=0)
        px_max, py_max = poly.max(axis=0)
        ix0 = max(0, int(np.floor((px_min - x0) / grid_res)))
        ix1 = min(nx, int(np.ceil((px_max - x0) / grid_res)))
        iy0 = max(0, int(np.floor((py_min - y0) / grid_res)))
        iy1 = min(ny, int(np.ceil((py_max - y0) / grid_res)))
        grid[iy0:iy1, ix0:ix1] = True
    return float(grid.mean())
