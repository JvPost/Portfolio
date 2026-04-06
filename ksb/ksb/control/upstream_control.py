# ksb/control/upstream_control.py

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List, Tuple

import numpy as np

from ksb.motion.trajectories import (
    CompositeTrajectory,
    ConstantJerkTrajectory,
    P, V, A,
)

# A single segment in the jerk timeline T: (t_start, jerk, duration)
Segment = Tuple[float, float, float]


class UpstreamController(ABC):
    """Stateful upstream belt controller backed by an append-only jerk timeline.

    The timeline T is a list of constant-jerk segments:
        T = [(t_n, j_n, T_n), ...]

    All kinematic quantities are derived from T by integration.
    Segments are never modified or removed — only appended.
    (Exception: _truncate_at discards speculative segments past a skip time.)
    """

    def __init__(self, vu: float) -> None:
        self._vu = vu
        self._timeline: List[Segment] = []
        self._t_end: float = 0.0
        self._last_t_spawn: float = -np.inf

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def subsection(
        self,
        t_spawn: float,
        distance: float,
    ) -> CompositeTrajectory:
        """Return a CompositeTrajectory covering exactly `distance` metres,
        starting at t_spawn under the current jerk timeline.

        The initial state [v, a] at t_spawn is derived internally from T.

        Args:
            t_spawn:  absolute spawn time of this input (s)
            distance: upstream control distance to cover (m)

        Returns:
            CompositeTrajectory in delta-position semantics (p starts at 0)
        """
        assert t_spawn > self._last_t_spawn, (
            f"Spawn times must be strictly increasing: "
            f"got {t_spawn} after {self._last_t_spawn}"
        )
        self._last_t_spawn = t_spawn

        x0 = self._state_at(t_spawn)

        segments: List[ConstantJerkTrajectory] = []
        remaining = distance
        t_now = t_spawn
        v_now = float(x0[V])
        a_now = float(x0[A])

        while remaining > 1e-10:
            self._ensure_covered(t_now, remaining, v_now)
            seg_t, seg_j, seg_T = self._segment_at(t_now)

            elapsed = t_now - seg_t
            seg_remaining = seg_T - elapsed

            x0_local = np.array([0.0, v_now, a_now])
            candidate = ConstantJerkTrajectory(x0=x0_local, T=seg_remaining, jerk=seg_j)
            dx_full = candidate.eval(seg_remaining)[P]

            if dx_full < remaining + 1e-9:
                segments.append(candidate)
                remaining -= dx_full
                t_now += seg_remaining
                end = candidate.eval(seg_remaining)
                v_now = float(end[V])
                a_now = float(end[A])
            else:
                dt = self._solve_distance(v_now, a_now, seg_j, remaining)
                x0_last = np.array([0.0, v_now, a_now])
                segments.append(ConstantJerkTrajectory(x0=x0_last, T=dt, jerk=seg_j))
                remaining = 0.0

        x0_traj = np.array([0.0, float(x0[V]), float(x0[A])])
        total_T = sum(s.T for s in segments)
        return CompositeTrajectory(
            x0=x0_traj,
            T=total_T,
            segments=tuple(segments),
        )

    @abstractmethod
    def on_skip(self, t_skip: float) -> None:
        """Notify the controller that a skip occurred at t_skip."""
        ...

    # ------------------------------------------------------------------
    # Timeline internals
    # ------------------------------------------------------------------

    def _append(self, t_start: float, jerk: float, duration: float) -> None:
        """Append one segment to the timeline."""
        if duration < 1e-12:
            return
        self._timeline.append((t_start, jerk, duration))
        self._t_end = t_start + duration

    def _ensure_covered(self, t: float, remaining_distance: float, v_now: float) -> None:
        """Extend the timeline with zero-jerk if it doesn't reach far enough.

        Uses v_now for the time estimate instead of v_u, since the belt may
        be traveling faster than v_u during an acceleration phase.
        """
        if t >= self._t_end:
            v_est = max(v_now, self._vu)
            needed = (t - self._t_end) + remaining_distance / v_est
            self._append(self._t_end, 0.0, needed)

    def _truncate_at(self, t: float) -> None:
        """Remove all timeline content after time t.

        Segments starting at or after t are removed entirely; the segment
        straddling t is shortened. Used when a skip interrupts a
        pre-appended acceleration profile.
        """
        while self._timeline and self._timeline[-1][0] >= t - 1e-12:
            self._timeline.pop()

        if self._timeline:
            seg_t, seg_j, seg_T = self._timeline[-1]
            seg_end = seg_t + seg_T
            if seg_end > t + 1e-12:
                self._timeline[-1] = (seg_t, seg_j, t - seg_t)

        self._t_end = t

    def _segment_at(self, t: float) -> Segment:
        """Return the segment that contains time t."""
        for seg in reversed(self._timeline):
            seg_t, seg_j, seg_T = seg
            if seg_t <= t + 1e-12:
                return seg
        raise ValueError(f"No segment found at t={t:.6f}")

    def _state_at(self, t: float) -> np.ndarray:
        """Return belt state [p, v, a] at absolute time t by integrating T."""
        p, v, a = 0.0, self._vu, 0.0
        for seg_t, seg_j, seg_T in self._timeline:
            if seg_t >= t:
                break
            dt = min(t - seg_t, seg_T)
            p += v * dt + 0.5 * a * dt**2 + (1 / 6) * seg_j * dt**3
            v += a * dt + 0.5 * seg_j * dt**2
            a += seg_j * dt
        return np.array([p, v, a])

    @staticmethod
    def _solve_distance(v0: float, a0: float, j: float, d: float) -> float:
        """Solve for smallest t > 0 such that v0*t + 0.5*a0*t² + (1/6)*j*t³ = d."""
        if abs(j) < 1e-12:
            if abs(a0) < 1e-9:
                return d / max(v0, 1e-10)
            a_c, b_c, c_c = 0.5 * a0, v0, -d
            disc = b_c**2 - 4 * a_c * c_c
            if disc < 0:
                raise ValueError("No real solution for distance")
            sq = np.sqrt(disc)
            candidates = [(-b_c + sq) / (2 * a_c), (-b_c - sq) / (2 * a_c)]
        else:
            roots = np.roots([j / 6.0, a0 / 2.0, v0, -d])
            candidates = [r.real for r in roots if abs(r.imag) < 1e-9 and r.real > 1e-9]

        if not candidates:
            raise ValueError("No positive real solution for distance")
        return float(min(candidates))


# ======================================================================
# Implementations
# ======================================================================


class ConstantVelocityControl(UpstreamController):
    """No-op controller. Belt runs at constant v_u. Baseline for comparison."""

    def __init__(self, vu: float) -> None:
        super().__init__(vu)

    def on_skip(self, t_skip: float) -> None:
        pass


class PreAccelerateControl(UpstreamController):
    """Upstream feedforward controller.

    Between skips the belt accelerates, building a velocity surplus that
    causes each input to arrive at the buffer entry earlier than it would
    at constant v_u.  On skip detection the belt decelerates back to
    (v_u, a=0), spending the surplus.

    The acceleration profile Pi_k^acc is an S-curve from (v_u, 0) up to
    (v_max_up, 0):

        [+j_max, t_ramp]   ramp a from 0 -> a_max_acc
        [0,      t_hold]   hold at a_max_acc (v climbing)
        [-j_max, t_ramp]   ramp a from a_max_acc -> 0

    After the S-curve completes, the belt cruises at v_max_up.
    _ensure_covered extends with zero-jerk, which maintains that cruise.

    If a skip fires mid-profile, _truncate_at removes the speculative
    future segments and the deceleration is sized to the actual (a_k, dv_k).

    The deceleration profile Pi_k^dec is sized by two quantities at s_k:

        a_k  = a(s_k)            acceleration at skip time  (m/s²)
        dv_k = v(s_k) - v_u      velocity surplus           (m/s)

    Two invariants must hold over each complete phase Pi_k:

        (1) sum j^(n) T^(n)                          = 0   (a -> 0)
        (2) sum [a_n T^(n) + 0.5 j^(n) (T^(n))^2]   = 0   (v -> v_u)

    Invariant 1 sizes the deceleration jerk ramps.
    Invariant 2 sizes the hold at -a_max.

    Pi_1 is not a special case: the acceleration profile is appended
    at t=0 in __init__, and the first on_skip call handles it identically
    to all subsequent skips.
    """

    def __init__(
        self,
        vu: float,
        j_max: float,
        a_max: float,
        a_max_acc: float,
        v_max_up: float,
    ) -> None:
        super().__init__(vu)
        self._j_max = j_max
        self._a_max = a_max              # deceleration bound (Pi_k^dec)
        self._a_max_acc = a_max_acc      # acceleration bound (Pi_k^acc)
        self._v_max_up = v_max_up
        self._dv_max = v_max_up - vu
        self._t_last_recovery: float = 0.0

        # Pi_1^acc starts immediately
        self._append_acceleration(0.0)

    # ------------------------------------------------------------------
    # Public
    # ------------------------------------------------------------------

    def on_skip(self, t_skip: float) -> None:
        """Handle skip event at t_skip.

        1. Read (a_k, dv_k) from belt state.
        2. Truncate speculative segments past t_skip.
        3. Append deceleration Pi_k^dec (if needed).
        4. Append acceleration Pi_{k+1}^acc.
        """
        state = self._state_at(t_skip)
        a_k = float(state[A])
        dv_k = float(state[V]) - self._vu

        self._truncate_at(t_skip)

        t_rec = t_skip
        if abs(a_k) > 1e-6 or abs(dv_k) > 1e-6:
            t_rec = self._append_deceleration(t_rec, a_k, dv_k)

        self._t_last_recovery = t_rec
        self._append_acceleration(t_rec)

    # ------------------------------------------------------------------
    # Acceleration profile
    # ------------------------------------------------------------------

    def _append_acceleration(self, t_start: float) -> None:
        """Eagerly append the full acceleration S-curve.

        Trapezoidal (when dv_max is large enough to reach a_max_acc):
            [+j_max, t_ramp]   ramp a: 0 -> a_max_acc
            [0,      t_hold]   hold at a_max_acc
            [-j_max, t_ramp]   ramp a: a_max_acc -> 0

        Triangular (when dv_max is small):
            [+j_max, t_r]      ramp a: 0 -> a_peak
            [-j_max, t_r]      ramp a: a_peak -> 0

        After this profile a = 0 and v = v_max_up. Subsequent
        _ensure_covered calls extend with zero-jerk cruise at v_max_up.
        """
        j = self._j_max
        a_acc = self._a_max_acc
        dv_max = self._dv_max

        t_ramp = a_acc / j
        dv_per_ramp = 0.5 * a_acc * t_ramp  # = a_acc² / (2·j)

        t_now = t_start

        if 2 * dv_per_ramp >= dv_max:
            # Triangular: dv_max reached before a_max_acc
            a_peak = np.sqrt(dv_max * j)
            t_r = a_peak / j
            self._append(t_now, +j, t_r)
            t_now += t_r
            self._append(t_now, -j, t_r)
        else:
            # Trapezoidal: ramp up -> hold -> ramp down
            dv_hold = dv_max - 2 * dv_per_ramp
            t_hold = dv_hold / a_acc
            self._append(t_now, +j, t_ramp)
            t_now += t_ramp
            self._append(t_now, 0.0, t_hold)
            t_now += t_hold
            self._append(t_now, -j, t_ramp)

    # ------------------------------------------------------------------
    # Deceleration profile
    # ------------------------------------------------------------------

    def _append_deceleration(
        self,
        t_start: float,
        a_k: float,
        dv_k: float,
    ) -> float:
        """Append Pi_k^dec segments to cancel a_k and dv_k.

        Returns t_k^rec — the time at which deceleration completes.

        The profile brings the belt from (v_u + dv_k, a_k) back to (v_u, 0).

        Trapezoidal (large dv_k, saturates at -a_max):
            [-j_max] ramp a_k -> -a_max
            [  0   ] hold at -a_max for t_hold
            [+j_max] ramp -a_max -> 0

        Triangular (small dv_k):
            [-j_max] ramp a_k -> -a_peak   (a_peak < a_max)
            [+j_max] ramp -a_peak -> 0

        Hold duration derivation (trapezoidal case):

            dv_ramps = 0.5 * (a_k² - 2·a_max²) / j
            t_hold   = (dv_k + dv_ramps) / a_max

        Triangular peak:

            a_peak = sqrt(0.5·a_k² + j·dv_k)
        """
        j = self._j_max
        a_max = self._a_max

        # Velocity change from trapezoidal ramps (without hold)
        dv_ramps = 0.5 * (a_k**2 - 2.0 * a_max**2) / j

        # Hold duration from velocity invariant
        t_hold = (dv_k + dv_ramps) / a_max

        t_now = t_start

        if t_hold > 1e-9:
            # ---- Trapezoidal ----
            T_down = (a_k + a_max) / j
            T_up = a_max / j

            self._append(t_now, -j, T_down)
            t_now += T_down
            self._append(t_now, 0.0, t_hold)
            t_now += t_hold
            self._append(t_now, +j, T_up)
            t_now += T_up
        else:
            # ---- Triangular ----
            a_peak_sq = 0.5 * a_k**2 + j * dv_k

            if a_peak_sq < 1e-12:
                return t_now

            a_peak = np.sqrt(a_peak_sq)

            T_down = (a_k + a_peak) / j
            T_up = a_peak / j

            self._append(t_now, -j, T_down)
            t_now += T_down
            self._append(t_now, +j, T_up)
            t_now += T_up

        return t_now