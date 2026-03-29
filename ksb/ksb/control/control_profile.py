from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List

import numpy as np

from ksb.motion.trajectories import CompositeTrajectory, ConstantJerkTrajectory, V, A, P


@dataclass(frozen=True)
class ControlProfile(ABC):
    @abstractmethod
    def subsection(
        self,
        t_start: float,
        x_start: np.ndarray,      # absolute state [p, v, a] at t_start
        distance_required: float,
    ) -> CompositeTrajectory:
        """Return a CompositeTrajectory covering exactly distance_required meters."""
        pass


@dataclass(frozen=True)
class ConstantJerkControl(ControlProfile):
    jerks: List[float]
    durations: List[float]
    loop: bool = True
    durations_cumsum: np.ndarray = field(default_factory=lambda: np.array([0.0]))

    def __post_init__(self):
        if len(self.jerks) != len(self.durations) or len(self.durations) == 0:
            raise ValueError("jerks and durations must be non-empty and equal length")
        cum = np.cumsum(self.durations)
        object.__setattr__(self, 'durations_cumsum', np.concatenate(([0.0], cum)))

    @property
    def cycle_duration(self) -> float:
        return self.durations_cumsum[-1]

    def subsection(
        self,
        t_start: float,
        x_start: np.ndarray,
        distance_required: float,
    ) -> CompositeTrajectory:
        if distance_required <= 0:
            return []

        segments: List[ConstantJerkTrajectory] = []
        remaining_distance = distance_required
        t_now = t_start
        current_v = float(x_start[V])
        current_a = float(x_start[A])

        while remaining_distance > 1e-10:
            t_mod = t_now % self.cycle_duration if self.loop else t_now
            phase_idx = np.searchsorted(self.durations_cumsum, t_mod, side='right') - 1
            if phase_idx < 0:
                phase_idx = 0

            j_phase = self.jerks[phase_idx]
            T_full = self.durations[phase_idx]

            x0_phase = np.array([0.0, current_v, current_a])
            phase_candidate = ConstantJerkTrajectory(x0=x0_phase, T=T_full, jerk=j_phase)

            dx_full = phase_candidate.end_state()[P]

            if dx_full < remaining_distance + 1e-9:
                segments.append(phase_candidate)
                remaining_distance -= dx_full
                t_now += T_full
                end = phase_candidate.end_state()
                current_v = end[V]
                current_a = end[A]
            else:
                dt = self._solve_remaining_time(
                    v0=current_v,
                    a0=current_a,
                    j=j_phase,
                    d_target=remaining_distance,
                    t_max=T_full,
                )
                x0_last = np.array([0.0, current_v, current_a])
                last_phase = ConstantJerkTrajectory(x0=x0_last, T=dt, jerk=j_phase)
                segments.append(last_phase)
                remaining_distance = 0.0
                break

        total_T = sum(s.T for s in segments)
        return CompositeTrajectory(
            x0=x_start,
            T=total_T,
            segments=tuple(segments),
        )

    @staticmethod
    def _solve_remaining_time(
        v0: float,
        a0: float,
        j: float,
        d_target: float,
        t_max: float,
    ) -> float:
        """Solve (1/6)*j*t³ + (1/2)*a0*t² + v0*t = d_target for smallest t in (0, t_max]."""
        if abs(j) < 1e-12:
            if abs(a0) < 1e-9:
                return d_target / max(v0, 1e-10) if v0 > 0 else t_max
            aa = 0.5 * a0
            bb = v0
            cc = -d_target
            disc = bb ** 2 - 4 * aa * cc
            if disc < 0:
                return t_max
            sqrt_disc = np.sqrt(disc)
            t1 = (-bb + sqrt_disc) / (2 * aa)
            t2 = (-bb - sqrt_disc) / (2 * aa)
            positives = [t for t in (t1, t2) if t > 1e-9]
            if not positives:
                return t_max
            return min(t for t in positives if t <= t_max + 1e-9) or t_max

        a = j / 6.0
        b = a0 / 2.0
        c = v0
        d = -d_target

        roots = np.roots([a, b, c, d])
        real_pos = [r.real for r in roots if np.isreal(r) and r.real > 1e-9]
        if not real_pos:
            return t_max

        candidates = [t for t in real_pos if t <= t_max + 1e-9]
        if not candidates:
            return t_max

        return min(candidates)
