# Section 2 — Surface Coverage Path Planning

Stack: ROS 2 Humble · MoveIt 2 · Python 3.10

## What's here

```
section2/
├── README.md                    ← this file
├── writeup.md                   ← coverage strategies, comparison + design notes
├── code/
│   ├── coverage_planner.py      ← raster + roundabout (countertop) + stacked half-circles (mirror)
│   ├── trajectory_builder.py    ← Cartesian → joint trajectory + time parameterization
│   ├── visualize_paths.py       ← matplotlib renderers
│   ├── plan_section2.py         ← orchestrator (run this)
│   └── replay_in_rviz.py        ← send the joint trajectories to /arm_controller for RViz viewing
└── outputs/
    ├── countertop_raster.png            ← top-down raster view
    ├── mirror_arcs.png                  ← front-on view of mirror stacked half-circle arcs
    ├── joint_trajectories.png           ← 6 joint angles vs time, both surfaces
    ├── coverage_metrics.png             ← coverage / path length / duration bar chart
    ├── trajectory_countertop.csv        ← per-waypoint (time, joints, xyz, pitch, yaw)
    ├── trajectory_mirror.csv            ← same for mirror
    ├── joint_trajectory_countertop.yaml ← ROS 2-shaped JointTrajectory message
    ├── joint_trajectory_mirror.yaml     ← same for mirror
    └── metrics.json                     ← machine-readable summary
```

## Constraints honored (from the spec)

| Constraint | Value |
|---|---|
| Tool pad footprint | 100 × 50 mm |
| Overlap | 15 % (mid of 10–20 % spec band) |
| Keep-out margin (edges) | 15 mm |
| Tool normal ≈ surface normal | within ±10° (pitch lattice ∈ {0°, ±5°, ±10°}) |
| Countertop speed | 0.20 m/s (mid of 0.15–0.25) |
| Mirror speed | 0.15 m/s (mid of 0.10–0.20) |

Additional planner-side keep-outs (introduced after RViz replay revealed
arm-body collisions even though the IK was solving):

| Exclusion | Where |
|---|---|
| Faucet disk | 10 cm radius around `(0, 0.50)` in XY (≈ pad + arm body clearance) |
| Centre-column strip | `|X| < 2.5 cm` for `Y > 0.30` on the countertop (the vertical column the faucet stands in) |
| Mirror top cap | wipe never goes above `Z = 0.45 m` (the topmost reachable rows have too few IK solutions and the wrist spins) |
| Mirror bottom-centre semicircle | 10 cm-radius semicircle centred at `(X = 0, Z = 0)` is excluded (right above the faucet column) |

## Build / run

```bash
# Section 1 prerequisites (kitchen scene + IKSolver + reachability heatmaps)
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch piper_wiping kitchen_full_launch.py    # MoveIt + RViz + kitchen
python3 ~/PiperWipe/section1/code/planning_scene.py &   # collision objects

# Optional: regenerate the mirror reachability map (~50 min, only needed once)
python3 ~/PiperWipe/section1/code/reachability_heatmap_mirror.py

# Section 2: plan, IK, time-parameterize, plot
python3 ~/PiperWipe/section2/code/plan_section2.py

# See it move: replay both trajectories on the arm controller in RViz
python3 ~/PiperWipe/section2/code/replay_in_rviz.py
```

## Design notes

- **Pad orientation rotates with sweep direction.** On both surfaces the
  pad's 100 mm long axis runs perpendicular to the sweep (horizontal pass
  → long axis along Z on the mirror / along Y on the counter). Keeps the
  swath width at a consistent 100 mm and the row pitch (85 mm) inside the
  15 % overlap spec.
- **Reachability masks gate the planner.** The countertop raster reads
  Section 1's `reachability.csv` and skips waypoints in unreachable cells
  *before* running IK; the mirror raster does the same against
  `reachability_mirror.csv`. This is what makes the mirror plan look like
  a half-dome — the upper 40 % of the panel is genuinely unreachable
  with the arm's current vertical extension, and the planner declines to
  even try.
- **IK joint-continuity seeding.** Each `/compute_ik` call is seeded with
  the previous waypoint's solution (`solver.solve(..., seed_joints=last_q)`).
  Without this seed the KDL solver picks the *nearest* branch independently
  per cell, which leads to wrist/elbow flips between adjacent waypoints
  and a juddery joint trajectory. With it, the trajectory is smooth in
  joint space — visible directly in `joint_trajectories.png`.
- **Faucet roundabout (in-plane).** `_segment_crosses_faucet()` checks every
  consecutive accepted waypoint pair (including inter-row transitions) and,
  if the straight line would drag the arm through the 10 cm-radius faucet
  disk, `_roundabout_around_faucet()` inserts a sequence of intermediate
  waypoints on a circle around the faucet centre at radius
  `FAUCET_KEEPOUT_R + 2 mm`. The tool stays IN CONTACT with the counter
  surface throughout — no lifting. Picks the shorter angular direction.
- **No custom .srv files.** Coverage planning reuses Section 1's
  `IKSolver` library directly; the resulting joint trajectories are
  serialized to a `trajectory_msgs/JointTrajectory`-shaped YAML and a
  flat CSV, which the RViz replay and Section 3's simulator both consume
  without needing message generation.
- **Why not MoveIt Cartesian planner (`computeCartesianPath`)?** It only
  produces *consecutive* waypoints that pass collision checks, so any cell
  that fails IK aborts the whole pass. Our approach plans the geometry
  first, then solves per-waypoint IK independently — when one cell fails,
  the path simply skips it (visualized in red) and continues. For a wipe
  routine on a known scene, this is strictly more robust.

See `writeup.md` for the headline metrics, the countertop-vs-mirror
comparison and the deeper trade-off discussion.
