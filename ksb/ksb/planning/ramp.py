"""Jerk-limited velocity-ramp primitive shared by the s-curve solver and
the registrar deceleration profile.

A ramp is a 3-phase transition v_a -> v_b under bounded jerk and
acceleration:

    Phase 1 (T1): jerk = sign * j_max, accel ramps from 0 to its peak
    Phase 2 (T2): jerk = 0,           accel held at peak (only if a_max binds)
    Phase 3 (T3): jerk = -sign * j_max, accel ramps back to 0

Triangular regime: T2 = 0 when |dv| < a_max^2 / j_max.
Trapezoidal regime: T2 > 0 when a_max binds.

`sign` encodes the direction: +1 for v_b > v_a (acceleration), -1 for
v_b < v_a (deceleration), 0 for v_a == v_b (degenerate).
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class RampKinematics:
    """Kinematics of a single v_a -> v_b transition under (j_max, a_max)."""
    T1: float
    T2: float
    T3: float
    displacement: float
    sign: float  # +1, -1, or 0

    @property
    def total_time(self) -> float:
        return self.T1 + self.T2 + self.T3


def ramp(v_a: float, v_b: float, j_max: float, a_max: float) -> RampKinematics:
    """Compute jerk-limited velocity-ramp kinematics from v_a to v_b.

    Returns durations of the three phases, total displacement, and the
    direction sign. Tolerances and formulae match the previous inlined
    implementations in SCurveSolver._ramp and
    RegistrarProfile._build_decel_segments byte-for-byte.
    """
    dv = abs(v_b - v_a)
    if dv < 1e-12:
        return RampKinematics(T1=0.0, T2=0.0, T3=0.0, displacement=0.0, sign=0.0)

    sign = 1.0 if v_b > v_a else -1.0
    dv_ramp_full = (a_max ** 2) / j_max

    use_a = dv >= dv_ramp_full - 1e-9
    if use_a:
        T1 = a_max / j_max
        dv_in = a_max * T1
        T2 = (dv - dv_in) / a_max
        T3 = T1
    else:
        T1 = math.sqrt(dv / j_max)
        T2 = 0.0
        T3 = T1

    j1 = sign * j_max
    x1 = v_a * T1 + (1.0 / 6.0) * j1 * T1 ** 3
    v1 = v_a + 0.5 * j1 * T1 ** 2
    a1 = j1 * T1
    x2 = v1 * T2 + 0.5 * a1 * T2 ** 2
    v2 = v1 + a1 * T2
    j3 = -sign * j_max
    x3 = v2 * T3 + 0.5 * a1 * T3 ** 2 + (1.0 / 6.0) * j3 * T3 ** 3

    return RampKinematics(
        T1=T1, T2=T2, T3=T3,
        displacement=x1 + x2 + x3,
        sign=sign,
    )
