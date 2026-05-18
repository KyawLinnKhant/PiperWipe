"""
wiping_controller.py — contact-aware wiping controller.

State machine:

    APPROACH       Tool descending toward surface, no contact yet.
                   Switch to FORCE when |F_normal| > F_TRIGGER (2 N).

    FORCE          Steady-state wiping. Maintain F_normal at the surface's
                   target (10 N counter / 6 N mirror) using an admittance
                   rule:
                       ż_cmd  =  Kp * (F_target − F_measured)
                                + Ki * ∫(F_target − F_measured) dt
                   The integral term removes the steady-state offset; the
                   anti-windup clamp keeps it sane during transients.
                   Switch to BACKOFF when |F_normal| > F_MAX (15 N).

    BACKOFF        Emergency lift: command the tool ±BACKOFF_STEP normal
                   to the surface and freeze the lateral path until
                   |F_normal| < F_TRIGGER again, then return to FORCE.

    OBSTACLE_AVOID Triggered when the next waypoint sits inside the faucet
                   keep-out radius (lookahead, not contact). Arc over by
                   lifting ARC_HEIGHT, traversing, then dropping back.

The "normal direction" is +Z for the countertop, −Y for the mirror, so the
controller produces a scalar normal-velocity command which the caller
applies to the right Cartesian axis.

This module is pure logic (no rclpy) so it can run in the sim or be wrapped
by a real ros2_control update loop.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

import numpy as np


# ── Spec-derived constants ───────────────────────────────────────────────────
F_TRIGGER = 2.0        # N — switch to force mode when |F| crosses this
F_MAX     = 15.0       # N — back off immediately above this

TARGET_FORCE = {"countertop": 10.0, "mirror": 6.0}      # N
TARGET_TOL   = {"countertop":  2.0, "mirror": 1.5}      # ± N
SPEED_RANGE  = {"countertop": (0.15, 0.25),             # m/s
                "mirror":     (0.10, 0.20)}
NOMINAL_SPEED = {"countertop": 0.20, "mirror": 0.15}

# ── Obstacle (faucet) ────────────────────────────────────────────────────────
FAUCET_XY  = (0.00, 0.50)
FAUCET_KEEPOUT_R = 0.055    # inflated radius (matches Section 2 keep-out)
ARC_HEIGHT = 0.05           # m — lift over faucet
BACKOFF_STEP = 0.003        # m — single emergency-lift step

# ── Controller gains ─────────────────────────────────────────────────────────
KP = 0.0025      # m/s per N — converts force error into normal velocity
KI = 0.001       # m/s per N·s — integral term
I_CLAMP = 0.02   # m/s — anti-windup clamp on the integral contribution


class Mode(Enum):
    APPROACH = "approach"
    FORCE = "force"
    BACKOFF = "backoff"
    OBSTACLE_AVOID = "obstacle_avoid"


@dataclass
class ControllerState:
    mode: Mode = Mode.APPROACH
    integral: float = 0.0
    last_obstacle_active: bool = False


@dataclass
class ControlCommand:
    """Per-tick command emitted by the controller."""
    normal_velocity: float       # m/s along surface normal (signed, + = INTO surface)
    lateral_scale: float = 1.0   # multiplier on the planned tangent speed (0 = freeze)
    z_offset: float = 0.0        # extra OFFSET to apply (used during OBSTACLE_AVOID arc)
    mode: Mode = Mode.APPROACH


def faucet_proximity(xy: np.ndarray) -> float:
    """Return distance from xy to the faucet centre (m)."""
    return float(np.hypot(xy[0] - FAUCET_XY[0], xy[1] - FAUCET_XY[1]))


def update(state: ControllerState,
           surface: str,
           force_measured: float,
           tool_xy: np.ndarray,
           next_xy: np.ndarray,
           dt: float) -> ControlCommand:
    """One tick of the controller. Returns the command for this dt."""
    target = TARGET_FORCE[surface]

    # ── Look-ahead obstacle check (countertop only) ──────────────────────
    if surface == "countertop":
        obstacle_active = faucet_proximity(next_xy) < FAUCET_KEEPOUT_R
    else:
        obstacle_active = False

    # ── Mode transitions ────────────────────────────────────────────────
    if obstacle_active:
        state.mode = Mode.OBSTACLE_AVOID
    elif state.mode == Mode.OBSTACLE_AVOID and not obstacle_active:
        # Just cleared the obstacle — return to APPROACH (re-acquire contact)
        state.mode = Mode.APPROACH
        state.integral = 0.0
    elif abs(force_measured) > F_MAX:
        state.mode = Mode.BACKOFF
        state.integral = 0.0
    elif state.mode == Mode.BACKOFF and abs(force_measured) < F_TRIGGER:
        state.mode = Mode.FORCE
    elif state.mode == Mode.APPROACH and abs(force_measured) > F_TRIGGER:
        state.mode = Mode.FORCE
        state.integral = 0.0

    # ── Per-mode command ────────────────────────────────────────────────
    if state.mode == Mode.APPROACH:
        # Constant descent toward the surface until contact.
        cmd = ControlCommand(normal_velocity=0.020,   # 20 mm/s gentle approach
                             lateral_scale=0.0,        # don't sweep until in contact
                             mode=state.mode)
    elif state.mode == Mode.FORCE:
        err = target - force_measured
        state.integral += err * dt
        state.integral = float(np.clip(state.integral * KI, -I_CLAMP, I_CLAMP) / KI)
        v_norm = KP * err + KI * state.integral
        v_norm = float(np.clip(v_norm, -0.05, 0.05))   # ±50 mm/s safety
        cmd = ControlCommand(normal_velocity=v_norm,
                             lateral_scale=1.0,
                             mode=state.mode)
    elif state.mode == Mode.BACKOFF:
        cmd = ControlCommand(normal_velocity=-0.030,   # 30 mm/s retract
                             lateral_scale=0.0,
                             mode=state.mode)
    elif state.mode == Mode.OBSTACLE_AVOID:
        # Lift to ARC_HEIGHT above surface and keep moving laterally so the
        # tool arcs over the faucet. Reset integral.
        state.integral = 0.0
        cmd = ControlCommand(normal_velocity=0.0,
                             lateral_scale=0.6,         # slow down over the obstacle
                             z_offset=ARC_HEIGHT,
                             mode=state.mode)
    else:
        cmd = ControlCommand(normal_velocity=0.0, mode=state.mode)

    state.last_obstacle_active = obstacle_active
    return cmd
