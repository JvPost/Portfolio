from __future__ import annotations

from dataclasses import dataclass, field
from typing import Union

import numpy as np
from ruckig import ControlInterface, InputParameter, Result, Ruckig, Trajectory

from ksb.motion.trajectories import TrajectoryProfile, P, V, A
from ksb.planning.contracts import (
    IProfileSolver,
    InfeasibleError,
    InputError,
    J_MAX,
    A_MAX,
    V_MAX,
)


# ──────────────────────────────────────────────────────────────────────────────
# RuckigTrajectory
# ──────────────────────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class RuckigTrajectory(TrajectoryProfile):
    """TrajectoryProfile wrapping a ruckig.Trajectory (position mode, 1-DoF).

    Delta position convention: eval(0)[P] == 0, eval(T)[P] == pf - pi.
    This holds naturally because the Ruckig trajectory is computed with
    current_position=0 and target_position=pf-pi in the solver.

    Monotonicity assumption in find_time_at_position: velocity >= 0 throughout
    (π^I semantics). Bisection raises InputError if this is violated.
    """

    _traj: object = field(compare=False, repr=False)  # ruckig.Trajectory

    def eval(self, t: Union[float, np.ndarray]) -> np.ndarray:
        scalar = np.isscalar(t)
        t_arr = np.atleast_1d(np.asarray(t, dtype=float))
        N = len(t_arr)
        out = np.empty((3, N))
        for idx in range(N):
            p_vec, v_vec, a_vec = self._traj.at_time(float(t_arr[idx]))
            out[P, idx] = p_vec[0]
            out[V, idx] = v_vec[0]
            out[A, idx] = a_vec[0]
        if scalar:
            return out[:, 0]
        return out

    def get_duration(self) -> float:
        return self.T

    def check_bounds(self, bounds: np.ndarray, num_samples: int = 100) -> bool:
        # Ruckig respects the bounds it was given by construction.
        return True

    def find_time_at_position(self, p_target: float, tol: float = 1e-9) -> float | None:
        """Find time t in [0, T] where position == p_target, via bisection.

        Position is assumed to be monotonically non-decreasing (v >= 0
        throughout). Raises InputError if bisection detects non-monotonicity.
        Returns None if p_target is outside [p(0), p(T)].
        """
        T = self.T
        p0 = self._traj.at_time(0.0)[0][0]
        pT = self._traj.at_time(T)[0][0]

        if p_target < p0 - tol or p_target > pT + tol:
            return None
        if abs(p_target - p0) < tol:
            return 0.0
        if abs(p_target - pT) < tol:
            return T

        t_lo, t_hi = 0.0, T
        p_lo, p_hi = p0, pT

        for _ in range(60):
            t_mid = 0.5 * (t_lo + t_hi)
            p_mid = self._traj.at_time(t_mid)[0][0]

            # if not (p_lo - tol <= p_mid <= p_hi + tol):
            #     raise InputError(
            #         f"Non-monotonic position in RuckigTrajectory bisection: "
            #         f"p(t_lo={t_lo:.6f})={p_lo:.6f}, p(t_mid={t_mid:.6f})={p_mid:.6f}, "
            #         f"p(t_hi={t_hi:.6f})={p_hi:.6f}"
            #     )

            if abs(p_mid - p_target) < tol:
                return t_mid

            if p_mid < p_target:
                t_lo = t_mid
                p_lo = p_mid
            else:
                t_hi = t_mid
                p_hi = p_mid

        return 0.5 * (t_lo + t_hi)


# ──────────────────────────────────────────────────────────────────────────────
# RuckigSolver
# ──────────────────────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class RuckigSolver(IProfileSolver):
    """Ruckig-based profile solver using ControlInterface.Position (1-DoF).

    Computes a minimum-time trajectory from (pi, vi, 0) to (pf, vf, 0)
    padded to exactly T seconds via minimum_duration.

    Delta position convention: current_position=0, target_position=pf-pi.
    The returned RuckigTrajectory lives in the same delta frame.

    Infeasibility signal: Ruckig always computes T_min. When T_min > T
    (requested horizon too small), traj.duration > T and InfeasibleError
    is raised. Callers such as get_next_slot catch this and advance to the
    next slot.
    """

    def solve(
        self, pi, vi, pf, vf, T, bounds, policy, ai=0, af=0
    ) -> RuckigTrajectory:
        Xf = pf - pi

        otg = Ruckig(1)
        inp = InputParameter(1)
        inp.control_interface = ControlInterface.Position

        inp.current_position = [0.0]
        inp.current_velocity = [float(vi)]
        inp.current_acceleration = [float(ai)]

        inp.target_position = [float(Xf)]
        inp.target_velocity = [float(vf)]
        inp.target_acceleration = [float(af)]

        inp.max_velocity = [float(bounds[V_MAX])]
        inp.max_acceleration = [float(bounds[A_MAX])]
        inp.max_jerk = [float(bounds[J_MAX])]

        inp.minimum_duration = float(T)

        traj = Trajectory(1)
        result = otg.calculate(inp, traj)

        if result != Result.Working:
            raise InfeasibleError(
                f"Ruckig failed with result={result} for "
                f"(pi={pi}, vi={vi}, pf={pf}, vf={vf}, T={T})"
            )

        # T_min > T: the requested horizon is shorter than the minimum trajectory.
        if traj.duration > T + 1e-9:
            raise InfeasibleError(
                f"Ruckig T_min={traj.duration:.9f} > T={T:.9f}: "
                f"infeasible for (pi={pi}, vi={vi}, pf={pf}, vf={vf})"
            )

        return RuckigTrajectory(
            x0=np.array([0.0, float(vi), ai]),
            T=traj.duration,
            _traj=traj,
        )
