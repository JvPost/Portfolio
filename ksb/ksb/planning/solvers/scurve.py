from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ksb.motion.trajectories import CompositeTrajectory, ConstantJerkTrajectory, V, A
from ksb.planning.contracts import IProfileSolver, InfeasibleError, J_MAX, A_MAX, V_MAX


@dataclass(frozen=True)
class SCurveSolver(IProfileSolver):
    """
    7-phase jerk-limited S-curve solver — fixed-time, analytical + scalar bisection.

    Profile shape: vi → v_peak → vf

    The 7 phases:
        1: jerk = +j_sign * j_max   (acc ramp, direction toward v_peak)
        2: jerk = 0                 (constant accel, may be zero-duration)
        3: jerk = -j_sign * j_max   (acc ramp down to zero)
        4: jerk = 0                 (coast at v_peak, may be zero-duration)
        5: jerk = -j_sign2 * j_max  (decel ramp, direction toward vf)
        6: jerk = 0                 (constant decel, may be zero-duration)
        7: jerk = +j_sign2 * j_max  (decel ramp down to zero)

    Strategy (fixed-time):
        Given (vi, vf, pf, T_m), find v_peak in [min(vi,vf), v_max] such that:
            X_acc(vi, v_peak) + v_peak * T_cruise + X_dec(v_peak, vf) = pf
            T_acc + T_cruise + T_dec = T_m
        f(v_peak) = X_acc + X_dec + v_peak*(T_m - T_acc - T_dec) - pf = 0
        Single bisection finds the root (f is monotone in v_peak).
    """

    min_duration: float = 1e-9

    def solve(self, pi, vi, pf, vf, T, bounds, policy) -> CompositeTrajectory:
        # pi is the starting position; the polynomial is expressed relative to pi=0
        Xf = pf  # total displacement to cover (pi assumed 0)
        T_m = T

        j_max = bounds[J_MAX]
        a_max = bounds[A_MAX]
        v_max = bounds[V_MAX]

        if T_m <= 0:
            raise InfeasibleError("T_m must be > 0")
        if Xf < 0:
            raise InfeasibleError("pf must be >= 0")
        if vi < 0 or vf < 0:
            raise InfeasibleError("vi and vf must be >= 0")
        if vi > v_max + 1e-9 or vf > v_max + 1e-9:
            raise InfeasibleError("Initial or final velocity exceeds V_max")

        dv_ramp_full = (a_max ** 2) / j_max  # dv covered by two jerk ramps at a_max

        def _ramp(v_a: float, v_b: float) -> tuple[float, float]:
            """Time and displacement for velocity transition v_a → v_b."""
            dv = abs(v_b - v_a)
            sign = 1.0 if v_b >= v_a else -1.0
            if dv < 1e-12:
                return 0.0, 0.0
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
            return T1 + T2 + T3, x1 + x2 + x3

        def _f(vp: float) -> tuple[float, float]:
            """Displacement residual and T_cruise for a given v_peak."""
            Ta, Xa = _ramp(vi, vp)
            Td, Xd = _ramp(vp, vf)
            T_cruise = T_m - Ta - Td
            return Xa + vp * T_cruise + Xd - Xf, T_cruise

        # ------------------------------------------------------------------
        # T_min: time-optimal profile
        # ------------------------------------------------------------------
        Ta_vm, Xa_vm = _ramp(vi, v_max)
        Td_vm, Xd_vm = _ramp(v_max, vf)
        X_no_cruise = Xa_vm + Xd_vm

        if X_no_cruise <= Xf + 1e-9:
            T_min = Ta_vm + (Xf - X_no_cruise) / v_max + Td_vm
        else:
            v_lo = min(vi, vf)
            v_hi = v_max
            for _ in range(64):
                vm = 0.5 * (v_lo + v_hi)
                _, Xa = _ramp(vi, vm)
                _, Xd = _ramp(vm, vf)
                if Xa + Xd < Xf:
                    v_lo = vm
                else:
                    v_hi = vm
                if v_hi - v_lo < 1e-10:
                    break
            vp0 = 0.5 * (v_lo + v_hi)
            Ta0, _ = _ramp(vi, vp0)
            Td0, _ = _ramp(vp0, vf)
            T_min = Ta0 + Td0

        if T_m < T_min - 1e-9:
            raise InfeasibleError(f"T_m={T_m:.6f} < T_min={T_min:.6f}")

        # ------------------------------------------------------------------
        # Find v_peak for fixed T_m via bisection
        # ------------------------------------------------------------------
        v_lo = min(vi, vf)
        v_hi = v_max

        _, tc_lo = _f(v_lo)
        if tc_lo < -1e-9:
            raise InfeasibleError(
                f"T_m={T_m:.4f} too short even for minimum v_peak={v_lo:.4f} "
                f"(T_cruise={tc_lo:.4f} < 0)"
            )

        f_lo, _ = _f(v_lo)
        f_hi, _ = _f(v_hi)

        if abs(f_lo) < 1e-9:
            v_peak = v_lo
        elif abs(f_hi) < 1e-9:
            v_peak = v_hi
        elif f_lo * f_hi > 0:
            raise InfeasibleError(
                f"No root in v_peak range [{v_lo:.4f}, {v_hi:.4f}]: "
                f"f_lo={f_lo:.4f}, f_hi={f_hi:.4f}"
            )
        else:
            for _ in range(64):
                vm = 0.5 * (v_lo + v_hi)
                fv, _ = _f(vm)
                if fv < 0:
                    v_lo = vm
                else:
                    v_hi = vm
                if v_hi - v_lo < 1e-10:
                    break
            v_peak = 0.5 * (v_lo + v_hi)

        Ta, _ = _ramp(vi, v_peak)
        Td, _ = _ramp(v_peak, vf)
        T_cruise = max(0.0, T_m - Ta - Td)

        # ------------------------------------------------------------------
        # Phase durations
        # ------------------------------------------------------------------
        def _durations(v_a: float, v_b: float) -> tuple[float, float, float]:
            dv = abs(v_b - v_a)
            if dv < 1e-12:
                return 0.0, 0.0, 0.0
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
            return T1, T2, T3

        t1, t2, t3 = _durations(vi, v_peak)
        t5, t6, t7 = _durations(v_peak, vf)
        t4 = T_cruise

        sign1 = 1.0 if v_peak >= vi else -1.0
        sign2 = 1.0 if vf >= v_peak else -1.0

        phase_jerk = [
            (t1,  sign1 * j_max),
            (t2,  0.0),
            (t3, -sign1 * j_max),
            (t4,  0.0),
            (t5,  sign2 * j_max),
            (t6,  0.0),
            (t7, -sign2 * j_max),
        ]

        # ------------------------------------------------------------------
        # Build ConstantJerkTrajectory segments
        # ------------------------------------------------------------------
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
        x0_composite = np.array([0.0, vi, 0.0])
        composite = CompositeTrajectory(
            x0=x0_composite,
            T=T_actual,
            segments=tuple(segments),
        )


        return composite
