# Section 3 — Contact-Aware Wiping Control

Stack: Python 3.10 (no ROS dependency at runtime — consumes Section 2's CSVs)

## What's here

```
section3/
├── README.md                    ← this file
├── writeup.md                   ← controller logic, mode transitions, design notes
├── code/
│   ├── ft_sensor_sim.py         ← Hooke's-law contact model → simulated wrist F/T
│   ├── wiping_controller.py     ← state machine + admittance PI law
│   ├── run_wiping_demo.py       ← drives the sim over Section 2's trajectories
│   ├── plot_results.py          ← static tracking plots (force / velocity / mode)
│   └── animate_demo.py          ← top-down GIF/MP4 (obstacle event highlight)
└── outputs/
    ├── wipe_log_<surface>.csv          ← 100 Hz log: t, mode, xyz, force, v_cmd, …
    ├── <surface>_tracking.png          ← force/velocity/mode timeline
    ├── obstacle_event_closeup.png      ← zoom on the arc-over event
    ├── wiping_demo.gif                 ← animated demo (faucet arc-over)
    ├── wipe_rviz.mp4                   ← RViz screen capture of the wipe (Deliverable 3)
    └── trajectory_countertop_naive.csv ← synthetic trajectory that crosses the faucet
```

## Demo video (Deliverable 3)

RViz playback of the Section 2 joint trajectories executing on
`/arm_controller/follow_joint_trajectory` against the live MoveIt 2 +
kitchen scene:

<video src="https://github.com/KyawLinnKhant/PiperWipe/raw/main/section3/outputs/wipe_rviz.mp4" controls muted width="100%"></video>

If the inline player doesn't load, download
[`outputs/wipe_rviz.mp4`](outputs/wipe_rviz.mp4).

## Spec compliance (Section 3 requirements)

| Task | Where it lives |
|---|---|
| 1. Switch to force control when \|Fz\| > 2 N | `wiping_controller.update()` `Mode.APPROACH → Mode.FORCE` on `|F| > F_TRIGGER` |
| 2. Maintain target force; back off if \|Fz\| > 15 N | PI admittance in `Mode.FORCE`; `Mode.BACKOFF` triggered on `|F| > F_MAX` |
| 3. Skip / locally replan around obstacle | `Mode.OBSTACLE_AVOID` arcs over the faucet keep-out radius |
| 4. Log + plot force vs time and velocity vs time | `run_wiping_demo.py` writes CSV; `plot_results.py` makes the PNGs |

Spec-derived parameters (set in `wiping_controller.py`):

| Constant | Value | Source |
|---|---|---|
| `F_TRIGGER` | 2 N | task 1 |
| `F_MAX` | 15 N | task 2 |
| `TARGET_FORCE["countertop"]` | 10 N (±2 N) | spec |
| `TARGET_FORCE["mirror"]` | 6 N (±1.5 N) | spec |
| `NOMINAL_SPEED["countertop"]` | 0.20 m/s | mid of 0.15–0.25 |
| `NOMINAL_SPEED["mirror"]` | 0.15 m/s | mid of 0.10–0.20 |
| `KP`, `KI` | 0.0025 m/s/N, 0.001 m/s/N·s | tuned for ~1 s contact-rise |
| `ARC_HEIGHT` | 0.05 m | enough to clear faucet base |
| `BACKOFF_STEP` | 3 mm | small per-tick retract |

## Headline results

| Run | Samples | force-mode in-band | mean F (force mode) |
|---|---:|:-:|:-:|
| Countertop (Section 2 trajectory) | 2837 | **2657 / 2681 (99.1 %)** | 10.00 N (target 10 ±2) |
| Mirror (Section 2 trajectory) | 3117 | **2943 / 2960 (99.4 %)** | 6.00 N (target 6 ±1.5) |
| Countertop naïve through faucet (demo) | 1359 | 481 / 556 (86.5 %)† | 9.53 N |

† 86.5 % is during steady wiping only — the run spends 49 % of its time in
APPROACH (re-acquiring contact after each obstacle arc-over) and 10 % in
OBSTACLE_AVOID itself. See `obstacle_event_closeup.png` for the per-event view.

## Build / run

The Section 3 sim has **no ROS dependency at runtime** — it reads the CSV
trajectories produced by Section 2 and writes plots / CSVs into `outputs/`.

```bash
# Prerequisite: Section 2's trajectory CSVs must exist
ls ~/PiperWipe/section2/outputs/trajectory_*.csv

# Three steps:
cd ~/PiperWipe/section3/code
python3 run_wiping_demo.py        # writes outputs/wipe_log_*.csv  (~2 s)
python3 plot_results.py           # writes 3 tracking PNGs + obstacle closeup
python3 animate_demo.py           # writes wiping_demo.gif (or .mp4 if ffmpeg)
```

## Design notes

- **Why admittance (PI on force → velocity) rather than impedance?** The
  Piper does not expose torque-control interfaces — only position/velocity.
  An admittance loop converts the force error into a *velocity command*,
  which is the right interface for stock joint-velocity controllers. The
  outer position controller closes the loop. (A real deployment would put
  this at a high rate ~500 Hz; the sim runs at 100 Hz, which is the floor
  for stable contact control with a 2000 N/m surface stiffness.)
- **Why Hooke's-law contact?** The spec only constrains `|Fz|`. A linear
  spring `F = k * penetration` reproduces the steady-state behavior any
  admittance controller actually sees, with two parameters to tune
  (`k_countertop`, `k_mirror`) — chosen so the target force corresponds to
  ~5 mm and ~4 mm of penetration respectively (realistic for a soft wipe pad).
- **OBSTACLE_AVOID uses lookahead, not contact.** The controller checks
  the *next-but-one* waypoint against the faucet keep-out radius and lifts
  to `ARC_HEIGHT` *before* the tool reaches the obstacle. This is the "skip
  or replan locally" branch from the spec; an alternative would be to wait
  until contact then retract, but at 0.20 m/s the tool would crash 4 mm
  past the faucet edge before the BACKOFF mode could react.
- **Why a separate naïve trajectory for the demo?** Section 2's planner
  already pre-filters the faucet keep-out so the real trajectory never
  exercises the OBSTACLE_AVOID branch. To prove the controller can handle
  an obstacle when one *does* appear (e.g. perception adding a new object
  at runtime), I generate a synthetic two-sweep trajectory that runs
  straight through the faucet zone and show the controller arc-over in
  `obstacle_event_closeup.png` and `wiping_demo.gif`.
- **Integral anti-windup.** The PI integral is clamped to ±`I_CLAMP / KI`
  so prolonged transients (e.g. during BACKOFF) don't accumulate windup
  that explodes when the loop closes again.

## Limitations / what a hardware port would need

- **Surface model is 1-D.** Real wiping has lateral friction (the `Fx, Fy`
  channels of an F/T sensor) which the controller would also need to track
  to keep the tool from skidding. Out of scope here.
- **No tool orientation feedback.** The controller acts on the normal-direction
  velocity only; rotational compliance (so the tool can sit flush on a
  warped surface) would be the next step.
- **Cartesian → joint mapping is open-loop.** A hardware port would feed
  `v_norm_cmd` through a Jacobian to joint velocities and use joint-velocity
  controllers. The Section 2 IK trajectory is the "outer" position reference.
- **Speed parameterization is constant.** The `lateral_scale` knob slows
  the tool during OBSTACLE_AVOID but doesn't yet ramp through accel limits.

See `writeup.md` for the state-machine walk-through and the per-plot
analysis.
