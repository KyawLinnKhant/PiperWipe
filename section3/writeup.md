# Section 3 — Contact-aware wiping: state machine + results

## The controller in one paragraph

We run an **admittance loop** (force error → velocity command) on the
surface-normal axis only. The lateral motion follows the planned Cartesian
tangent from Section 2 at the per-surface nominal speed. A small
state-machine wraps the admittance law so the controller can:

1. **Approach** the surface (constant 20 mm/s descent) until the F/T
   sensor crosses the 2 N trigger.
2. **Hold** the target force (10 N counter / 6 N mirror) with a PI rule:
   `ż_cmd = Kp·(F_target − F_meas) + Ki·∫err dt`, anti-wind-up clamped.
3. **Back off** at 30 mm/s if the measured force ever exceeds the 15 N
   safety cap, returning to FORCE once back under the 2 N trigger.
4. **Arc over** a foreseen obstacle (the faucet's 55 mm keep-out radius)
   by lifting 5 cm above the surface, traversing at 60 % lateral speed,
   then dropping back to re-acquire contact.

Mode transitions are all driven by `|F_normal|` thresholds + an explicit
lookahead obstacle check on the *next-but-one* waypoint.

## Headline results

### Countertop (Section 2 trajectory)

![countertop tracking](outputs/countertop_tracking.png)

- 28.4 s total, ~1.5 s of APPROACH, then sustained FORCE for the rest.
- **99.1 %** of force-mode samples are inside the 10 ± 2 N band.
- Mean force during FORCE: **10.00 N** (target 10). Std dev under 0.3 N
  once the integrator has settled (~ 0.5 s into FORCE mode).
- Lateral speed is held flat at 0.20 m/s — inside the 0.15–0.25 m/s spec
  range. Normal command stays ≈ 0 once the integrator catches the small
  steady-state error from the 0.05 N sensor noise.

### Mirror (Section 2 trajectory)

![mirror tracking](outputs/mirror_tracking.png)

- 31.2 s total, similar shape — 1.4 s APPROACH then steady FORCE.
- **99.4 %** in the 6 ± 1.5 N band; mean **6.00 N**.
- Lateral speed held at 0.15 m/s, mid of the 0.10–0.20 m/s spec.
- Slightly tighter tracking than the countertop because the lower target
  force + lower surface stiffness give a more forgiving control margin.

### Obstacle event (naïve trajectory)

![obstacle closeup](outputs/obstacle_event_closeup.png)

The naïve trajectory has two horizontal sweeps at Y = 0.46 m and Y = 0.50 m,
both crossing X = 0 where the faucet sits. The plot shows two complete
arc-overs:

1. **t ≈ 2.8 s** — Tool in FORCE mode, holding 10 N. Lookahead sees the
   next-but-one waypoint inside the keep-out radius → OBSTACLE_AVOID.
   Tool snaps up to 50 mm above the counter, force drops to 0.
2. **t ≈ 3.5 s** — Past the faucet → APPROACH. Tool descends, hits
   counter at t ≈ 5.8 s (Z back to 0 mm), force ramps to target → FORCE.
3. **t ≈ 8.5 s** — Second sweep approaches the faucet again, same arc-over
   plays out symmetrically.

Each arc-over costs ≈ 3 seconds (rise + traverse + descend + re-acquire);
the controller is intentionally conservative on the re-acquire (gentle
descent) to avoid the contact transient overshooting the 15 N safety cap.

### The full animated demo

![demo gif](outputs/wiping_demo.gif)

Top-down view of the naïve trajectory with a live force gauge. The
trail behind the tool is coloured by mode (yellow = approach, green =
force-tracking, purple = obstacle arc-over). The two purple gaps over
the faucet are the OBSTACLE_AVOID events. The force gauge bar tracks
the target band (green shaded region) once contact is established.

## State machine in detail

```
        +────────────+
        | APPROACH   |  z descends at 20 mm/s,  lateral frozen
        +────────────+
              | |F| > F_TRIGGER (2 N)
              ↓
        +────────────+
        |   FORCE    |  PI(F_err) → ż_cmd, lateral at nominal v
        +────────────+
        |     ↑    |
   |F|>15│     │|F|<2
        ↓     |
        +────────────+
        |  BACKOFF   |  z retracts at 30 mm/s, lateral frozen
        +────────────+

OBSTACLE_AVOID can supersede any of the above when the next-but-one
waypoint is inside the faucet keep-out — lifts to ARC_HEIGHT, traverses
at 0.6× nominal lateral, then drops to APPROACH on the other side.
```

Why this layering:
- **APPROACH is necessary, not just a startup phase.** Every BACKOFF /
  OBSTACLE_AVOID event leaves the tool above the surface, so re-acquiring
  contact has to share the same code path.
- **OBSTACLE_AVOID is a lookahead, not a reactive event.** Reacting on
  contact would be too late at 0.20 m/s — the gripper body would smash
  into the faucet in ~50 ms after the F/T trigger.
- **BACKOFF freezes lateral motion** so the tool doesn't continue dragging
  through whatever surface anomaly just spiked the force.

## Gain tuning

`KP = 0.0025 m/s/N`, `KI = 0.001 m/s/N·s` produce a contact rise (0 → in-band)
in ~0.5 s on both surfaces. Higher KP gave faster rise but visible
oscillation in the simulated noise; lower gave 2-3 second rises that
ate into the time budget for short trajectories. The integral term is
essential — without it the steady-state force sits 0.5–1 N below target
because the surface stiffness needs continuous force-error to maintain
penetration.

Anti-windup: `I_CLAMP = 0.02 m/s` on the integral *output* prevents
windup blowing up during the seconds-long BACKOFF / OBSTACLE_AVOID
phases where the integrator would otherwise grow unbounded.

## Why a separate naïve demo trajectory?

Section 2's planner already pre-filters cells inside the faucet keep-out
so the *real* trajectory never even tries to wipe near the faucet — and
therefore the OBSTACLE_AVOID branch never fires on it. That's correct
behaviour for an offline planner with full scene knowledge, but it leaves
the runtime obstacle handler untested.

The naïve trajectory (`outputs/trajectory_countertop_naive.csv`) is a
synthetic two-row raster that ignores the keep-out — exactly what would
arrive at the controller if (a) the planner didn't know about the faucet,
(b) perception added the faucet at runtime, or (c) the scene moved. Running
the controller on it produces the visible arc-overs in the close-up and
GIF, which is the deliverable the spec asks for.

## Limitations

- **1-D contact only.** The controller acts on the surface-normal force,
  not lateral friction. Real wipes need a `Fx, Fy` channel and either
  velocity-mode tracking or a Jacobian-projection back to the same
  admittance law.
- **No tool-orientation compliance.** A real arm would let the wrist
  comply rotationally so the pad sits flush on a slightly warped surface;
  Section 3 keeps the tool orientation fixed from Section 2.
- **Sim only, no MoveIt round-trip.** The sim integrates Cartesian
  position directly instead of feeding `ż_cmd` through `compute_ik` at
  every tick. A hardware port would either (a) run the admittance loop
  at ~500 Hz inside ros2_control and let MoveIt own the joint trajectory,
  or (b) use MoveIt servo with a force-modified Cartesian setpoint.
- **Fixed surface stiffness.** Real countertops vs glass differ in
  stiffness by an order of magnitude. The two `k_*` parameters in
  `ft_sensor_sim.py` would need to be measured per-surface in a real
  deployment, or estimated online from the F/x slope during APPROACH.
