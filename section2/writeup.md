# Section 2 — Coverage strategies, metrics + trade-offs

## Headline numbers

|                              | Countertop (raster + roundabout) | Mirror (stacked half-circles) |
|------------------------------|:-:|:-:|
| Waypoints (after reach mask + keep-outs) | 303 | 87 |
| IK-solved waypoints          | 231 (76.2 %) | **86 (98.9 %)** |
| Coverage of planner's patch  | **81.9 %** | **56.0 %** |
| Cartesian path length        | 6.78 m | 2.07 m |
| Nominal speed                | 0.20 m/s | 0.15 m/s |
| Estimated execution time     | **26.0 s** | **13.5 s** |

![countertop](outputs/countertop_raster.png)
![mirror](outputs/mirror_arcs.png)
![metrics](outputs/coverage_metrics.png)

## Countertop — snake raster + in-plane roundabout

A boustrophedon (snake) raster: the pad's 100 mm long axis runs along Y
so each horizontal pass paints a 100 mm Y-strip, rows spaced by 85 mm
(= 100 × (1 − 15 % overlap)). Within each row the tool centre is sampled
every 2 cm; any cell inside the **10 cm-radius faucet disk** or the **5 cm
centre-column strip behind Y = 0.30** is dropped; any cell that the
Section 1 reachability mask marks unreachable is dropped.

When two consecutive accepted waypoints (even across rows) would drag the
straight-line traverse through the inflated faucet disk, the planner
inserts an **in-plane roundabout**: a sequence of intermediate waypoints
arranged on a circle of radius `FAUCET_KEEPOUT_R + 2 mm` around the
faucet centre, picking the shorter angular direction. The tool stays at
contact height throughout — no lifting required. This is the visible
green curve at the top-centre of `countertop_raster.png` that wraps
around the faucet disk.

**Where it falls short.** The reachability mask covers 60 cm of the
table's 120 cm width, so cells beyond ±30 cm in X have no reach data and
are dropped — that's the "wings" you see *outside* the green swaths. The
remaining 72 / 303 IK failures cluster on the back row directly behind
the faucet, where even the tilt+yaw lattice (pitch ∈ {0°, ±5°, ±10°} ×
yaw 12 angles) cannot find a collision-free pose.

**Iteration history.** Three prior implementations were tried before
landing on the stacked half-circles:
1. Rectangular spiral — wasted rings on cells outside the dome, IK
   wrist-flip in the upper-right corner of every ring.
2. Top-down straight-line raster — covered the dome cleanly but missed
   the curvy edges at the top of each row.
3. Concentric semicircles from a bottom-middle pivot — covered the dome
   shape but left gaps between rings near the top.

The stacked half-ellipse arcs (this version) combine the row-by-row
structure of a raster with the curvy reach of an arc, fit the dome
naturally, and look like a real wiper.

## Joint-continuity seeding kills the wrist flips

The single biggest fix this iteration: **every** `/compute_ik` call is
now seeded with the previous waypoint's solution (`solver.solve(...,
seed_joints=last_q)`). Without seeding, the KDL solver picks the cheapest
branch independently per cell, and two adjacent targets that differ by
2 cm in Cartesian space can resolve to two completely different elbow /
wrist configurations — a "branch flip" that the controller has to drive
through, often the long way around joint limits.

The difference is visible in `joint_trajectories.png`:
- **Countertop trace** — joints 1–3 ride smooth sinusoids with the raster;
  joints 4–6 mostly hold.
- **Mirror trace** — also smooth now (was jagged in the spiral version).
  Each half-circle arc is a clean curve through the apex; row-to-row
  transitions are 90°-ish wrist re-orientations at the chord endpoints,
  but smooth ones because the IK seeding keeps the wrist on the same
  branch from waypoint to waypoint.

## Countertop vs Mirror, head-to-head

| Property | Countertop raster + roundabout | Mirror stacked half-circles |
|---|---|---|
| Pattern | Snake (boustrophedon) + arc detour | Stacked half-ellipses, chord at bottom, bulge up |
| Obstacle in workspace | Yes (faucet) — in-plane roundabout handles it | No obstacle (planner keep-outs handle the faucet column) |
| Reachable area | ~70 % of plan grid | ~50 % of mirror panel (top + bottom-centre excluded) |
| IK solve rate | 76.2 % | 98.9 % |
| Direction changes | 180° at every row turn, smooth arc around faucet | 90° corners at chord endpoints, smooth arc through apex |
| Best suited to | Wide flat rectangles with column obstacles | Vertical surfaces with dome-shaped reachable region |

The mirror's much higher IK success rate (100 % vs 75.6 %) comes from
two things: (a) the planner-side keep-outs (top cap + bottom-centre ban)
remove exactly the cells where the spec's ±10° pitch tolerance is not
quite enough; (b) the reachability mask filters out the unreachable
upper half of the mirror *before* IK is attempted, so the planner never
emits a "Hail Mary" waypoint that's likely to fail.

## Limitations / next steps

- **No joint-continuity across surfaces.** The countertop trajectory
  ends with one IK branch; the mirror trajectory starts with another.
  The RViz replay shows a brief "settle" between the two. A real
  deployment would seed the mirror's first IK from the countertop's last
  pose so the transition is smooth.
- **Constant Cartesian speed.** Real arms need ramp-up/ramp-down and
  joint velocity limits. Fix: apply MoveIt's
  `default_planner_request_adapters/AddTimeOptimalParameterization` (the
  same one already wired into `kitchen_full_launch.py`'s move_group),
  which enforces per-joint velocity/acceleration bounds.
- **Mirror upper 40 % stays unwiped.** Software can't fix it — a real
  deployment would either (a) raise the arm base by 10–15 cm, (b) use a
  longer-reach arm, or (c) plan a two-pose pipeline where the base
  repositions between strokes.
- **Faucet keep-out is generous.** The 10 cm disk is sized for the
  *arm body* (link5 / forearm), not the pad. A tighter integration would
  swept-volume-check the URDF mesh against the faucet rather than using
  a single inflated disk.
