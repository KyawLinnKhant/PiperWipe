"""
ft_sensor_sim.py — simulated wrist force/torque sensor.

The real Piper does not ship with an F/T sensor; for Section 3 we model the
sensor reading from first principles:

  * Surfaces (countertop, mirror) behave as linear springs along their
    normal direction:  F_normal = k * penetration_depth, where penetration
    is how far the tool TIP sits past the surface plane (≥ 0 only).
  * No friction, no torque — the spec only cares about |Fz|.
  * Optional Gaussian noise approximates real-sensor jitter.

Surface frame convention (matches the planner):
  countertop normal = world +Z  → penetration = surface_top_z − tool_tip_z
  mirror normal     = world -Y  → penetration = tool_tip_y − mirror_face_y

The class exposes a single `read(surface, tool_tip)` returning a scalar
F_normal in Newtons (we call it "Fz" everywhere to match the spec language,
even when the contact normal is world -Y for the mirror).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


# Scene geometry — matches kitchen_scene.py
COUNTERTOP_TOP_Z = 0.05
MIRROR_FACE_Y = 0.555 - 0.005   # front face of the 1-cm-thick mirror panel


@dataclass
class FTSensor:
    """Linear spring contact model along the surface normal."""

    k_countertop: float = 2000.0     # N/m  (≈ 5 mm penetration for 10 N target)
    k_mirror:     float = 1500.0     # N/m  (≈ 4 mm penetration for 6 N target)
    noise_std:    float = 0.05       # N (Gaussian); set 0 to disable
    rng:          np.random.Generator = None

    def __post_init__(self):
        if self.rng is None:
            self.rng = np.random.default_rng(seed=42)

    def read(self, surface: str, tool_tip_xyz: np.ndarray) -> float:
        """Return the signed normal-direction force in N.

        Positive = compressive (tool pushing into surface). Zero when no contact.
        """
        x, y, z = float(tool_tip_xyz[0]), float(tool_tip_xyz[1]), float(tool_tip_xyz[2])
        if surface == "countertop":
            penetration = COUNTERTOP_TOP_Z - z
            k = self.k_countertop
        elif surface == "mirror":
            penetration = y - MIRROR_FACE_Y
            k = self.k_mirror
        else:
            raise ValueError(f"unknown surface {surface!r}")

        if penetration <= 0:
            f = 0.0
        else:
            f = k * penetration

        if self.noise_std > 0 and f > 0:
            f += float(self.rng.normal(0.0, self.noise_std))
        return f
