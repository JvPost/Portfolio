"""S-curve solver: 7-phase jerk-limited fixed-time trajectory.

Profile family: vi → v_m → vf via two jerk-limited ramps and a cruise phase.
The cruise velocity v_m can be above both endpoints (peak), below both (dip),
or between them (monotone) — all share one code path because the ramp signs
are derived from (v_a, v_b), not hard-coded.

Bisection over v_m ∈ [v_min, V_max] finds the unique root of:
    f(v_m) = ramp_in.displacement + v_m * T_cruise + ramp_out.displacement - Xf = 0

f is monotone-increasing in v_m over the full range: larger v_m → more
displacement at fixed T (whether the profile dips or peaks). A single
bisection therefore covers every case.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ksb.motion.trajectories import CompositeTrajectory, ConstantJerkTrajectory, V, A
from ksb.planning.contracts import IProfileSolver, InfeasibleError, J_MAX, A_MAX, V_MAX
from ksb.planning.ramp import ramp


@dataclass(frozen=True)
class SCurveSolver(IProfileSolver):
    """7-phase jerk-limited S-curve solver — fixed-time, analytical + bisection.

    Uses the shared `ramp` primitive for each half of the profile so that
    dip-shaped profiles (v_m < min(vi, vf)) are handled identically to
    peak-shaped ones.  Exposes `feasibility_window` so callers can bound
    the search before invoking `solve`.
    """

    min_duration: float = 1e-9

    # ------------------------------------------------------------------
    # Feasibility window
    # ------------------------------------------------------------------

    def feasibility_window(
        self, pi, vi, pf, vf, bounds, policy
    ) -> tuple[float, float]:
        """Return (T_min, T_max) for which solve(..., T, ...) is feasible.

        T_min: time-optimal — cruise at V_max.
        T_max: time-permissive — cruise at v_min (inf when v_min == 0).

        If the geometry is infeasible at any T (e.g. Xf < 0), returns
        (math.inf, 0.0) as a sentinel (T_min > T_max signals infeasibility).
        """
        j_max = bounds[J_MAX]
        a_max = bounds[A_MAX]
        v_max = bounds[V_MAX]
        v_min = policy.v_min
        Xf = pf - pi

        if Xf < 0 or vi < 0 or vf < 0:
            return (math.inf, 0.0)

        T_min = self._t_min(vi, vf, Xf, j_max, a_max, v_max, v_min)
        T_max = self._t_max(vi, vf, Xf, j_max, a_max, v_min)
        return (T_min, T_max)

    # ------------------------------------------------------------------
    # Primary interface
    # ------------------------------------------------------------------

    def solve(self, pi, vi, pf, vf, T, bounds, policy) -> CompositeTrajectory:
        Xf = pf - pi

        j_max = bounds[J_MAX]
        a_max = bounds[A_MAX]
        v_max = bounds[V_MAX]
        v_min = policy.v_min

        if T <= 0:
            raise InfeasibleError("T must be > 0")
        if Xf < 0:
            raise InfeasibleError("pf must be >= pi")
        if vi < 0 or vf < 0:
            raise InfeasibleError("vi and vf must be >= 0")
        if vi > v_max + 1e-9 or vf > v_max + 1e-9:
            raise InfeasibleError("Initial or final velocity exceeds V_max")

        T_min = self._t_min(vi, vf, Xf, j_max, a_max, v_max, v_min)
        T_max = self._t_max(vi, vf, Xf, j_max, a_max, v_min)

        tol = 1e-9
        if T < T_min - tol:
            raise InfeasibleError(
                f"T too small: T={T:.6f} < T_min={T_min:.6f}"
            )
        if T > T_max + tol:
            raise InfeasibleError(
                f"T too large: T={T:.6f} > T_max={T_max:.6f}"
            )

        v_m = self._bisect_vm(vi, vf, Xf, T, j_max, a_max, v_max, v_min)
        return self._build(vi, vf, v_m, T, j_max, a_max)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _t_min(
        self,
        vi: float, vf: float, Xf: float,
        j_max: float, a_max: float, v_max: float, v_min: float,
    ) -> float:
        """Time-optimal T: cruise at V_max if displacement permits."""
        rk_in = ramp(vi, v_max, j_max, a_max)
        rk_out = ramp(v_max, vf, j_max, a_max)
        X_ramps = rk_in.displacement + rk_out.displacement

        if X_ramps <= Xf + 1e-9:
            # Fit a cruise segment at V_max to cover the remaining distance.
            return rk_in.total_time + (Xf - X_ramps) / v_max + rk_out.total_time

        # Ramps at V_max overshoot: find the largest v_m ≤ V_max such that
        # the two ramps just fit inside Xf (no cruise phase).
        v_lo, v_hi = v_min, v_max
        for _ in range(64):
            vm = 0.5 * (v_lo + v_hi)
            X = ramp(vi, vm, j_max, a_max).displacement + ramp(vm, vf, j_max, a_max).displacement
            if X < Xf:
                v_lo = vm
            else:
                v_hi = vm
            if v_hi - v_lo < 1e-10:
                break
        vm_star = 0.5 * (v_lo + v_hi)
        return ramp(vi, vm_star, j_max, a_max).total_time + ramp(vm_star, vf, j_max, a_max).total_time

    def _t_max(
        self,
        vi: float, vf: float, Xf: float,
        j_max: float, a_max: float, v_min: float,
    ) -> float:
        """Time-permissive T: cruise at v_min.

        If v_min == 0, T_max = inf (arbitrarily slow coast at 0 m/s).

        If the two ramps at v_min already overshoot Xf, the trajectory
        cannot slow below a higher v_m — T_max equals the no-cruise ramp
        time at the smallest feasible v_m (unusual configuration).
        """
        if v_min <= 0.0:
            return math.inf

        rk_in = ramp(vi, v_min, j_max, a_max)
        rk_out = ramp(v_min, vf, j_max, a_max)
        X_ramps = rk_in.displacement + rk_out.displacement

        if X_ramps <= Xf + 1e-9:
            return rk_in.total_time + (Xf - X_ramps) / v_min + rk_out.total_time

        # Ramps to v_min overshoot: find the smallest v_m ≥ v_min such that
        # ramps still fit in Xf.  T_max is the no-cruise time at that v_m.
        v_lo, v_hi = v_min, max(vi, vf)
        for _ in range(64):
            vm = 0.5 * (v_lo + v_hi)
            X = ramp(vi, vm, j_max, a_max).displacement + ramp(vm, vf, j_max, a_max).displacement
            if X < Xf:
                v_hi = vm
            else:
                v_lo = vm
            if v_hi - v_lo < 1e-10:
                break
        vm_star = 0.5 * (v_lo + v_hi)
        return ramp(vi, vm_star, j_max, a_max).total_time + ramp(vm_star, vf, j_max, a_max).total_time

    def _f(
        self,
        v_m: float, vi: float, vf: float, Xf: float, T: float,
        j_max: float, a_max: float,
    ) -> float:
        """Displacement residual at cruise velocity v_m for fixed T."""
        rk_in = ramp(vi, v_m, j_max, a_max)
        rk_out = ramp(v_m, vf, j_max, a_max)
        T_cruise = T - rk_in.total_time - rk_out.total_time
        return rk_in.displacement + v_m * T_cruise + rk_out.displacement - Xf

    def _bisect_vm(
        self,
        vi: float, vf: float, Xf: float, T: float,
        j_max: float, a_max: float, v_max: float, v_min: float,
    ) -> float:
        """Bisect on v_m ∈ [v_min, V_max] to satisfy the displacement equation."""
        v_lo, v_hi = v_min, v_max
        f_lo = self._f(v_lo, vi, vf, Xf, T, j_max, a_max)
        f_hi = self._f(v_hi, vi, vf, Xf, T, j_max, a_max)

        if abs(f_lo) < 1e-9:
            return v_lo
        if abs(f_hi) < 1e-9:
            return v_hi

        # f must be monotone-increasing in v_m; a sign check guards against
        # numerical edge cases that would indicate a window-computation bug.
        assert f_lo <= f_hi + 1e-6, (
            f"f not monotone: f(v_min={v_lo:.4f})={f_lo:.4f} > f(V_max={v_hi:.4f})={f_hi:.4f}; "
            "feasibility window is inconsistent"
        )

        for _ in range(64):
            vm = 0.5 * (v_lo + v_hi)
            fv = self._f(vm, vi, vf, Xf, T, j_max, a_max)
            if fv < 0:
                v_lo = vm
            else:
                v_hi = vm
            if v_hi - v_lo < 1e-10:
                break
        return 0.5 * (v_lo + v_hi)

    def _build(
        self,
        vi: float, vf: float, v_m: float, T: float,
        j_max: float, a_max: float,
    ) -> CompositeTrajectory:
        """Assemble a CompositeTrajectory from the solved cruise velocity."""
        rk_in = ramp(vi, v_m, j_max, a_max)
        rk_out = ramp(v_m, vf, j_max, a_max)
        T_cruise = max(0.0, T - rk_in.total_time - rk_out.total_time)

        j_in = rk_in.sign * j_max
        j_out = rk_out.sign * j_max

        phase_jerk = [
            (rk_in.T1,  j_in),
            (rk_in.T2,  0.0),
            (rk_in.T3,  -j_in),
            (T_cruise,   0.0),
            (rk_out.T1,  j_out),
            (rk_out.T2,  0.0),
            (rk_out.T3,  -j_out),
        ]

        segments = []
        cur_v = vi
        cur_a = 0.0

        for duration, jerk in phase_jerk:
            if duration < self.min_duration:
                cur_v += cur_a * duration + 0.5 * jerk * duration ** 2
                cur_a += jerk * duration
                continue
            x0 = np.array([0.0, cur_v, cur_a])
            seg = ConstantJerkTrajectory(x0=x0, T=duration, jerk=jerk)
            end = seg.end_state()
            cur_v = end[V]
            cur_a = end[A]
            segments.append(seg)

        if not segments:
            raise InfeasibleError("No segments generated — degenerate profile.")

        T_actual = sum(seg.T for seg in segments)
        return CompositeTrajectory(
            x0=np.array([0.0, vi, 0.0]),
            T=T_actual,
            segments=tuple(segments),
        )
