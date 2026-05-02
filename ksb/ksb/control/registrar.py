from __future__ import annotations

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
from ksb.planning.ramp import ramp

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
        Lambda_R = self.L_reg - self.input_length # control length
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

        Uses the shared `ramp` primitive (ksb.planning.ramp), so the math is
        structurally consistent with any other consumer of the same primitive.
        """
        rk = ramp(self.v_in, self.v_out, self.j_max, self.a_max)
        if rk.sign == 0.0:
            return 0.0, 0.0, []

        T1, T2, T3 = rk.T1, rk.T2, rk.T3
        j1 = rk.sign * self.j_max
        j3 = -rk.sign * self.j_max

        # Intermediate states for segment construction
        v1 = self.v_in + 0.5 * j1 * T1 ** 2
        a1 = j1 * T1
        v2 = v1 + a1 * T2

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

        return rk.total_time, rk.displacement, segments
