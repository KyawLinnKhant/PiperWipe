# Section 1 — Where can / can't the arm reach, and why?

## Setup recap

- Arm: AgileX Piper (6-DOF), MoveIt 2, KDL/LMA numerical IK via `/compute_ik`.
- Base: `world → base_link` at `xyz = (0, 0, 0.05)`, `rpy = (0, 0, π/2)` —
  on the front edge of a 1.20 × 0.60 m countertop, facing +Y.
- Tool target = `link6` (tip of the arm chain in `piper_with_gripper_moveit`).
  Gripper fingers extend `0.1358 m` along link6's +Z, so for a tool-down wipe
  pose `link6.z = 0.05 + 0.1358 + 0.005 = 0.1908 m` (5 mm tip clearance).
- Patch: `X ∈ [-0.30, +0.30] m`, `Y ∈ [-0.05, +0.55] m`, 2 cm grid → 31 × 31 = 961 cells.
- **Per cell**, the heatmap searches an orientation lattice of `pitch ∈ {0°, ±5°, ±10°}` ×
  `yaw ∈ {0°, ±30°, ±60°, ±90°, ±120°, ±150°, 180°}` (60 combinations, first
  success wins). Pitch stays within the assignment's ±10° surface-normal
  tolerance; yaw spins the gripper about the tool's approach axis.
- `avoid_collisions=True` against the countertop, mirror, faucet base + spout
  published by `planning_scene.py` — i.e. the planner has to *actually* avoid
  the obstacles, not pretend they aren't there.

## Headline result

**784 / 961 cells (81.6 %) reachable.** All 177 failures come back as
`no_ik_solution` — with the obstacles in the scene and the orientation
lattice exhausted, the solver could not find a joint configuration.

![heatmap](outputs/reachability_heatmap.png)

## Where the arm CAN reach

- The **fully-covered arch is now `Y ∈ [0.07, 0.41]`** (vs `[0.07, 0.35]`
  for the naïve straight-down test) — 18 consecutive rows at 100 % across
  the entire 60 cm X span. That's the comfortable wipe zone.
- The back row **between Y = 0.43 and Y = 0.49 is mostly reachable** — this
  is the area immediately around the faucet base. Where straight-down was
  rejected for collision, a ≈ 5–10° pitch lets the gripper body pass
  *beside* the faucet cylinder.
- Of the 784 reachable cells:
  - **720 (92 %)** solve with the default 0° pitch — straight-down wipe.
  - **64 (8 %)** require a 5° or 10° pitch tilt — and 56 of those 64 sit at
    `Y ≥ 0.40`, confirming pitch is the move that unlocks the back-of-counter
    region against the faucet.
  - **15 cells (1.9 %)** require a yaw beyond ±90° (i.e. spinning the
    gripper more than a quarter-turn) — these are the cells immediately to
    the side of the faucet, where the gripper's wide face needs to point
    along Y to fit in the narrow Y-gap between the faucet cylinder and the
    workspace edge.
- Yaw distribution is left/right symmetric (517 cells solve at 0°, 138 at
  +30°, 78 at +60°, plus mirror counts on the negative side).

## Where the arm CAN'T reach (and why)

1. **Directly under itself** — the red footprint around `(0, 0)` extending
   roughly `X ∈ [-0.07, +0.07], Y ∈ [-0.03, +0.05]` (about 25 cells). With
   link6 commanded to `z = 0.191 m` and a near-vertical tool, the wrist
   would have to fold 180° back over its own base; joints 2/3/5 do not have
   the range, and even pitch ±10° doesn't recover it. **Standard 6-DOF
   kinematic blind spot**, not a scene issue.

2. **Behind the faucet / against the mirror (`Y ≥ 0.50`)** — the entire row
   at `Y = 0.51` and beyond is red. Three things stack here and even the
   pitch + yaw search can't squeeze through:
   - The mirror plane sits at `Y = 0.555`. With the gripper body ~5 cm
     wide in this orientation, link6 needs to be at least ~5 cm clear of
     the mirror to avoid the body intersecting it. That means link6's Y
     must be ≲ 0.505 → cells at `Y = 0.51+` have no admissible wrist pose.
   - The faucet spout (a 5 cm long box at `Y ∈ [0.45, 0.50]`, `Z ≈ 0.157 m`)
     sits in the path of any "lean over the faucet" trajectory — a 10°
     pitch isn't enough to clear it for cells directly behind.
   - Workspace radius: from base `(0, 0)` to `(0, 0.55)` is 55 cm, plus
     the tool stand-off — that is essentially the Piper's reach limit at
     this height.

3. **Single cell directly behind the faucet (`X = 0, Y = 0.43`)** — even
   with pitch + yaw the gripper body fouls the faucet spout. Move 2 cm
   sideways and it's green again, so this is a sharp pocket, not a band.

## How much did pitch + yaw actually buy us?

| Search strategy            | Reachable | Δ      | What it adds                          |
|----------------------------|-----------|--------|---------------------------------------|
| Straight-down, 7 yaws      | 711 / 961 | —      | naïve baseline                        |
| Pitch ±10°, yaw full circle | 784 / 961 | **+73 cells (+7.6 pp)** | back row around faucet, side-of-faucet, workspace-edge cells |

The remaining red is real — kinematic self-occlusion at the base, mirror
wall + faucet spout for the deepest row, and tiny pockets where two
obstacles overlap.

## Sanity checks

- All 177 failures report `no_ik_solution`. The scene is small enough that
  MoveIt's collision check passes quickly and the actual kinematic /
  collision-feasible search is what fails — i.e. the red regions are
  workspace + obstacle geometry walls, not solver timeouts.
- Per-X reachability is symmetric to within 1–2 cells about `X = 0`, as
  expected for a base oriented along +Y.

## Limitations

- **Single height.** The heatmap only checks `link6.z = 0.1908 m`
  (gripper tip ~5 mm above the counter). A real wipe needs to push the tip
  *onto* the counter — that 5 mm of clearance is what the Section 3 force
  controller is supposed to close, so kinematic reachability at this stand-off
  is a good proxy for wipe reachability.
- **Pitch tilt is two-direction.** The lattice tilts about the tool's local
  X-axis only; a more thorough search would also tilt about local Y. In
  practice the unreachable band at `Y ≥ 0.51` is bounded by the mirror
  wall, which no pitch direction can defeat, so this would not change the
  red region appreciably.
- **Solver budget.** Each (pitch, yaw) attempt gets 300 ms with KDL/LMA.
  TRAC-IK or a 1 s budget would shave a handful more cells off, mostly at
  the back-corner edges; the main red zones are scene-geometry-bound, not
  solver-bound.
