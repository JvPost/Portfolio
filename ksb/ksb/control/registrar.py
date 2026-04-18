from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List

import numpy as np

from ksb.motion.trajectories import (
    CompositeTrajectory,
    ConstantJerkTrajectory,
    LinearTrajectory,
    TrajectoryProfile,
    V,
    A,
)
from ksb.planning.contracts import InfeasibleError

_MIN_DURATION = 1e-9


@dataclass(frozen=True)
class RegistrarProfile:
    """Precomputed registrar trajectory: jerk-limited deceleration from v_in to v_out.

    The registrar is stateless — every input executes the identical trajectory.
    This object is computed once at construction and reused for the full batch.

    Parameters
    ----------
    v_in : float
        Entry velocity at the buffer/registrar boundary B^{BR} (m/s).
    v_out : float
        Exit velocity at the registrar/downstream boundary B^{RD} (m/s).
    L_reg : float
        Total registrar length (m).
    input_length : float
        Physical length of one input (m).
    j_max : float
        Maximum jerk magnitude (m/s³).
    a_max : float
        Maximum acceleration magnitude (m/s²).

    Exposed attributes
    ------------------
    trajectory : CompositeTrajectory
        Full registrar trajectory over effective control length Lambda_R.
    T_total : float
        Total transit time through the registrar (s).
    T_active : float
        Duration of the active deceleration phase (s).
    T_coast : float
        Duration of the constant-velocity coast at v_out (s).
    dp_active : float
        Displacement consumed by the active deceleration (m).
    """

    v_in: float
    v_out: float
    L_reg: float
    input_length: float
    j_max: float
    a_max: float

    # Derived — set in __post_init__ via object.__setattr__
    trajectory: CompositeTrajectory = field(init=False, repr=False)
    T_total: float = field(init=False)
    T_active: float = field(init=False)
    T_coast: float = field(init=False)
    dp_active: float = field(init=False)

    def __post_init__(self) -> None:
        Lambda_R = self.L_reg - self.input_length
        assert Lambda_R > 0, (
            f"Effective registrar length Lambda_R = L_reg - input_length "
            f"= {self.L_reg} - {self.input_length} = {Lambda_R:.4f} m must be positive"
        )

        T_active, dp_active, segments = self._build_decel_segments()

        if dp_active > Lambda_R + 1e-9:
            raise InfeasibleError(
                f"Registrar infeasible: deceleration requires {dp_active:.4f} m "
                f"but only {Lambda_R:.4f} m available. Increase L_registrar."
            )

        dp_coast = Lambda_R - dp_active
        T_coast = dp_coast / self.v_out if dp_coast > _MIN_DURATION * self.v_out else 0.0

        if T_coast > _MIN_DURATION:
            x0_coast = np.array([0.0, self.v_out, 0.0])
            coast_seg = LinearTrajectory(x0=x0_coast, T=T_coast)
            segments.append(coast_seg)

        if not segments:
            raise InfeasibleError("Registrar produced no trajectory segments.")

        T_actual = sum(seg.T for seg in segments)
        x0_traj = np.array([0.0, self.v_in, 0.0])
        traj = CompositeTrajectory(x0=x0_traj, T=T_actual, segments=tuple(segments))

        object.__setattr__(self, "trajectory", traj)
        object.__setattr__(self, "T_active", T_active)
        object.__setattr__(self, "T_coast", T_coast)
        object.__setattr__(self, "dp_active", dp_active)
        object.__setattr__(self, "T_total", T_actual)

    def _build_decel_segments(self) -> tuple[float, float, List[TrajectoryProfile]]:
        """Compute the 3-phase jerk-limited deceleration from v_in to v_out.

        Uses the same ramp kinematics as SCurveSolver._ramp(), so the math
        is guaranteed consistent with the buffer solver.

        Returns
        -------
        T_active : float
            Total duration of the active deceleration phase.
        dp_active : float
            Displacement consumed during deceleration.
        segments : list of TrajectoryProfile
            ConstantJerkTrajectory segments (1–3) representing the deceleration.
        """
        dv = self.v_in - self.v_out
        dv_ramp_full = self.a_max ** 2 / self.j_max  # dv where a_max is just reached

        if dv < 1e-12:
            return 0.0, 0.0, []

        use_a = dv >= dv_ramp_full - 1e-9  # trapezoidal — a_max binds
        if use_a:
            T1 = self.a_max / self.j_max
            dv_in_T1 = self.a_max * T1          # = a_max² / j_max = dv_ramp_full
            T2 = (dv - dv_in_T1) / self.a_max
            T3 = T1
        else:
            T1 = math.sqrt(dv / self.j_max)
            T2 = 0.0
            T3 = T1

        # Deceleration: jerk = -j_max (phase 1), 0 (phase 2), +j_max (phase 3)
        # Mirrors SCurveSolver._ramp with sign = -1 (v_b < v_a).
        j1 = -self.j_max
        j3 = +self.j_max

        # Intermediate state after phase 1
        v1 = self.v_in + 0.5 * j1 * T1 ** 2
        a1 = j1 * T1                            # negative; = -a_max if trapezoidal

        # Displacement accumulators (matching _ramp formula exactly)
        x1 = self.v_in * T1 + (1.0 / 6.0) * j1 * T1 ** 3

        # Intermediate state after phase 2
        v2 = v1 + a1 * T2
        x2 = v1 * T2 + 0.5 * a1 * T2 ** 2

        # Displacement in phase 3
        x3 = v2 * T3 + 0.5 * a1 * T3 ** 2 + (1.0 / 6.0) * j3 * T3 ** 3

        T_active = T1 + T2 + T3
        dp_active = x1 + x2 + x3

        # Build ConstantJerkTrajectory segments (skip zero-duration phases)
        segments: List[TrajectoryProfile] = []
        if T1 > _MIN_DURATION:
            segments.append(ConstantJerkTrajectory(
                x0=np.array([0.0, self.v_in, 0.0]),
                T=T1,
                jerk=j1,
            ))
        if T2 > _MIN_DURATION:
            segments.append(ConstantJerkTrajectory(
                x0=np.array([0.0, v1, a1]),
                T=T2,
                jerk=0.0,
            ))
        if T3 > _MIN_DURATION:
            segments.append(ConstantJerkTrajectory(
                x0=np.array([0.0, v2, a1]),
                T=T3,
                jerk=j3,
            ))

        return T_active, dp_active, segments
