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
    Segments are only ever added as far forward as needed.
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
            self._ensure_covered(t_now, remaining)
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
        self._timeline.append((t_start, jerk, duration))
        self._t_end = t_start + duration
    
    def _ensure_covered(self, t: float, remaining_distance: float) -> None:
        if t >= self._t_end:
            needed = (t - self._t_end) + remaining_distance / self._vu
            self._append(self._t_end, 0.0, needed)

    def _segment_at(self, t: float) -> Segment:
        """Return the segment that contains time t."""
        for seg in reversed(self._timeline):
            seg_t, seg_j, seg_T = seg
            if seg_t <= t:
                return seg
        raise ValueError(f"No segment found at t={t:.6f}")

    def _state_at(self, t: float) -> np.ndarray:
        """Return belt state [p, v, a] at absolute time t by integrating T."""
        p, v, a = 0.0, self._vu, 0.0
        for seg_t, seg_j, seg_T in self._timeline:
            if seg_t >= t:
                break
            dt = min(t - seg_t, seg_T)
            p += v * dt + 0.5 * a * dt**2 + (1/6) * seg_j * dt**3
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


class ConstantJerkControl(UpstreamController):
    def __init__(self, vu: float) -> None:
        super().__init__(vu)

    def on_skip(self, t_skip: float) -> None:
        pass


class DecelerateOnSkipControl(UpstreamController):
    def __init__(
        self,
        vu: float,
        j_max: float,
        a_max: float,
        a_max_acc: float,
    ) -> None:
        super().__init__(vu)
        self._j_max = j_max
        self._a_max = a_max
        self._a_max_acc = a_max_acc
        self._t_last_recovery: float = 0.0

    def on_skip(self, t_skip: float) -> None:
        A_k = self._compute_accumulated_area(self._t_last_recovery, t_skip)

        if abs(A_k) < 1e-9:
            return

        j_max = self._j_max
        a_max = self._a_max
        t_ramp = a_max / j_max
        A_ramps = a_max * t_ramp

        t_now = t_skip

        if A_k <= A_ramps:
            a_peak = np.sqrt(A_k * j_max)
            t_ramp_short = a_peak / j_max
            self._append(t_now, -j_max, t_ramp_short)
            t_now += t_ramp_short
            self._append(t_now, +j_max, t_ramp_short)
            t_now += t_ramp_short
        else:
            t_hold = (A_k - A_ramps) / a_max
            self._append(t_now, -j_max, t_ramp)
            t_now += t_ramp
            self._append(t_now, 0.0, t_hold)
            t_now += t_hold
            self._append(t_now, +j_max, t_ramp)
            t_now += t_ramp

        self._t_last_recovery = t_now

    def _compute_accumulated_area(self, t_start: float, t_end: float) -> float:
        A = 0.0
        for seg_t, seg_j, seg_T in self._timeline:
            seg_end = seg_t + seg_T
            overlap_start = max(seg_t, t_start)
            overlap_end = min(seg_end, t_end)
            if overlap_end > overlap_start:
                A += seg_j * (overlap_end - overlap_start)
        return A