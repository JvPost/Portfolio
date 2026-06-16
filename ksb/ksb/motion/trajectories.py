from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from functools import cached_property
from typing import Optional, Tuple, Union

import numpy as np
from scipy.optimize import brentq

from ksb.planning.contracts import J_MAX, A_MAX, V_MAX

# ---------- State vector indices ----------
# x = np.array([p, v, a])  where p is position (m), v velocity (m/s), a acceleration (m/s²)
P, V, A = 0, 1, 2


@dataclass(frozen=True)
class TrajectoryProfile(ABC):
    x0: np.ndarray  # Initial state vector [p, v, a]; p is always 0 (delta semantics)
    T: float        # Duration [s]

    @cached_property
    def xf(self) -> np.ndarray:
        return self.eval(self.T)

    @abstractmethod
    def eval(self, t: Union[float, np.ndarray]) -> np.ndarray:
        """Return state array at time t.

        Scalar t  → shape (3,)   = [p, v, a]
        Array  t  → shape (3, N) = [[p...], [v...], [a...]]

        Position is relative to the start of this segment (delta semantics).
        """
        pass

    @abstractmethod
    def get_duration(self) -> float:
        """Total duration T [s]."""
        pass

    @abstractmethod
    def check_bounds(self, bounds: np.ndarray, num_samples: int = 100) -> bool:
        """True if trajectory stays within bounds array [j_max, A_max, V_max, gap_min]."""
        pass

    @abstractmethod
    def find_time_at_position(self, p_target: float, tol: float = 1e-9) -> float | None:
        pass

    def __call__(self, t: Union[float, np.ndarray]) -> np.ndarray:
        return self.eval(t)


# ---------- Polynomial (quintic) trajectory ----------
@dataclass(frozen=True)
class PolynomialTrajectory(TrajectoryProfile):
    poly: np.poly1d  # position polynomial (high-to-low coefficients)

    def __post_init__(self):
        if not np.isclose(self.poly(0), 0, atol=1e-6):
            raise ValueError("Polynomial must start at p=0 (delta position semantics)")

    def eval(self, t: Union[float, np.ndarray]) -> np.ndarray:
        scalar = np.isscalar(t)
        t_arr = np.atleast_1d(np.asarray(t, dtype=float))

        p = self.poly(t_arr)
        v = self.poly.deriv(1)(t_arr)
        a = self.poly.deriv(2)(t_arr)

        if scalar:
            return np.array([float(p[0]), float(v[0]), float(a[0])])
        return np.array([p, v, a])  # shape (3, N)

    def get_duration(self) -> float:
        return self.T

    def check_bounds(self, bounds: np.ndarray, num_samples: int = 100) -> bool:
        t = np.linspace(0, self.T, num_samples)
        state = self.eval(t)  # shape (3, N)
        j = self.poly.deriv(3)(t)
        return (
            np.all(np.abs(state[V]) <= bounds[V_MAX])
            and np.all(np.abs(state[A]) <= bounds[A_MAX])
            and np.all(np.abs(j) <= bounds[J_MAX])
        )

    def find_time_at_position(self, p_target: float, tol: float = 1e-9, maxiter: int = 40) -> float | None:
        """Find time t where position = p_target (relative to segment start).

        Returns None if p_target is outside the reachable range.
        """
        p_start = self.x0[P]  # 0.0 (delta semantics)
        p_target_abs = p_target + p_start
        p_end = self.poly(self.T)

        if p_target_abs < p_start - tol or p_target_abs > p_end + tol:
            return None

        if abs(p_target_abs - p_start) < tol:
            return 0.0
        if abs(p_target_abs - p_end) < tol:
            return self.T

        def residual(t: float) -> float:
            return self.poly(t) - p_target_abs

        try:
            return brentq(residual, a=0.0, b=self.T, xtol=tol, rtol=1e-12,
                          maxiter=maxiter, full_output=False)
        except ValueError:
            return None


# ---------- Constant-velocity trajectory ----------
@dataclass(frozen=True)
class LinearTrajectory(TrajectoryProfile):
    """Constant-velocity segment. x0[P] must be 0, x0[A] must be 0."""

    def __post_init__(self):
        if abs(self.x0[A]) > 1e-8:
            raise ValueError("LinearTrajectory requires zero initial acceleration")
        if not np.isclose(self.x0[P], 0, atol=1e-6):
            raise ValueError("LinearTrajectory must start at Δp=0 (delta position semantics)")

    def eval(self, t: Union[float, np.ndarray]) -> np.ndarray:
        scalar = np.isscalar(t)
        t_arr = np.atleast_1d(np.asarray(t, dtype=float))
        t_c = np.clip(t_arr, 0, self.T)

        p = self.x0[V] * t_c
        v = np.full_like(t_arr, self.x0[V])
        a = np.zeros_like(t_arr)

        if scalar:
            return np.array([float(p[0]), float(v[0]), float(a[0])])
        return np.array([p, v, a])  # shape (3, N)

    def get_duration(self) -> float:
        return self.T

    def check_bounds(self, bounds: np.ndarray, num_samples: int = 100) -> bool:
        return (
            abs(self.x0[V]) <= bounds[V_MAX]
            and 0 <= bounds[A_MAX]
            and 0 <= bounds[J_MAX]
        )

    def find_time_at_position(self, p_target: float, tol: float = 1e-9) -> float | None:
        """Find time t where position = p_target (relative to segment start)."""
        p_end = self.x0[V] * self.T
        if p_target < -tol or p_target > p_end + tol:
            return None
        if abs(self.x0[V]) < 1e-12:
            return None
        return float(np.clip(p_target / self.x0[V], 0.0, self.T))


# ---------- Constant-jerk primitive ----------
@dataclass(frozen=True)
class ConstantJerkTrajectory(TrajectoryProfile):
    """Single constant-jerk phase: cubic position, quadratic velocity, linear acceleration.

    x0[P] must be 0 (delta position semantics).
    Kinematics:
        p(t) = x0[V]*t + 0.5*x0[A]*t² + (1/6)*jerk*t³
        v(t) = x0[V] + x0[A]*t + 0.5*jerk*t²
        a(t) = x0[A] + jerk*t
    """
    jerk: float

    def __post_init__(self):
        if abs(self.T) < 1e-9:
            raise ValueError("Duration T must be positive")
        if not np.isclose(self.x0[P], 0.0, atol=1e-6):
            raise ValueError("ConstantJerkTrajectory expects x0[P] ≈ 0 (delta position semantics)")

    def eval(self, t: Union[float, np.ndarray]) -> np.ndarray:
        scalar = np.isscalar(t)
        t_arr = np.atleast_1d(np.asarray(t, dtype=float))
        t_c = np.clip(t_arr, 0.0, self.T)

        t2 = t_c ** 2
        t3 = t_c ** 3

        if self.jerk > 0:
            assert True

        p = self.x0[V] * t_c + 0.5 * self.x0[A] * t2 + (1.0 / 6.0) * self.jerk * t3
        v = self.x0[V] + self.x0[A] * t_c + 0.5 * self.jerk * t2
        a = self.x0[A] + self.jerk * t_c

        if scalar:
            return np.array([float(p[0]), float(v[0]), float(a[0])])
        return np.array([p, v, a])  # shape (3, N)

    def get_duration(self) -> float:
        return self.T

    def check_bounds(self, bounds: np.ndarray, num_samples: int = 100) -> bool:
        if abs(self.jerk) > bounds[J_MAX] + 1e-9:
            return False

        a_start = self.x0[A]
        a_end = self.x0[A] + self.jerk * self.T
        if np.any(np.abs([a_start, a_end]) > bounds[A_MAX] + 1e-9):
            return False

        v_start = self.x0[V]
        v_end = self.eval(self.T)[V]
        v_candidates = [v_start, v_end]

        if abs(self.jerk) > 1e-9:
            t_crit = -self.x0[A] / self.jerk
            if 0 < t_crit < self.T:
                v_candidates.append(self.eval(t_crit)[V])

        if np.any(np.abs(v_candidates) > bounds[V_MAX] + 1e-9):
            return False

        return True

    def end_state(self) -> np.ndarray:
        """State array [p, v, a] at t = T."""
        return self.eval(self.T)

    def find_time_at_position(self, p_target: float, tol: float = 1e-9) -> float | None:
        """Find time t where position = p_target (relative to segment start).

        Solves: v0*t + 0.5*a0*t² + (1/6)*j*t³ = p_target
        Returns None if p_target is outside [0, p(T)].
        """
        p_end = self.eval(self.T)[P]

        if p_target < -tol or p_target > p_end + tol:
            return None
        if abs(p_target) < tol:
            return 0.0
        if abs(p_target - p_end) < tol:
            return self.T

        v0, a0, j = self.x0[V], self.x0[A], self.jerk
        roots = np.roots([j / 6.0, a0 / 2.0, v0, -p_target])
        candidates = [
            r.real for r in roots
            if abs(r.imag) < 1e-9 and 0.0 < r.real < self.T + tol
        ]
        if not candidates:
            return None
        return float(min(candidates))


# ---------- Composite trajectory ----------
@dataclass(frozen=True)
class CompositeTrajectory(TrajectoryProfile):
    segments: Tuple[TrajectoryProfile, ...]
    continuity_check_start: int = 1

    def __post_init__(self):
        if len(self.segments) == 0:
            raise ValueError("CompositeTrajectory must have at least one segment")

        # Velocity & acceleration continuity at junctions
        for i in range(self.continuity_check_start, len(self.segments)):
            prev_seg = self.segments[i-1]
            curr_seg = self.segments[i]

            prev_xf = prev_seg.xf
            curr_x0 = curr_seg.x0
            if not np.allclose(
                [prev_xf[V]],
                [curr_x0[V]],
                atol=1e-4,
            ):
                raise ValueError(
                    f"Velocity or acceleration discontinuity between segment {i-1} and {i}\n"
                    f"  prev end: v={prev_xf[V]:.6f}, a={prev_xf[A]:.6f}\n"
                    f"  next start: v={curr_x0[V]:.6f}, a={curr_x0[A]:.6f}"
                )

        computed_t = sum(seg.T for seg in self.segments)
        if not np.isclose(self.T, computed_t, atol=1e-6):
            raise ValueError(
                f"Provided T ({self.T}) does not match sum of segment durations ({computed_t})"
            )

        computed_p = sum(seg.xf[P] for seg in self.segments)
        __p = self.xf[P]
        if not np.isclose(__p, computed_p, atol=1e-6):
            raise ValueError(
                f"Provided P ({__p}) does not match sum of segment distances ({computed_p})"
            )
            

    def eval(self, t: Union[float, np.ndarray]) -> np.ndarray:
        if np.isscalar(t):
            return self._evaluate_scalar(t)
        return self._evaluate_array(np.asarray(t, dtype=float))

    def _evaluate_scalar(self, t: float) -> np.ndarray:
        t = max(0.0, min(t, self.T))

        if t == 0.0:
            x0 = self.segments[0].x0
            return np.array([x0[P], x0[V], x0[A]])

        cum_time = 0.0
        cum_p_offset = 0.0

        for seg in self.segments:
            seg_end_time = cum_time + seg.T
            if t <= seg_end_time:
                local_t = t - cum_time
                local = seg.eval(local_t)  # shape (3,)
                return np.array([cum_p_offset + local[P], local[V], local[A]]) # don't loop through all segments
            full = seg.eval(seg.T)
            cum_p_offset += full[P]
            cum_time = seg_end_time

        # looped through all segments
        last = self.segments[-1].eval(self.segments[-1].T)
        return np.array([cum_p_offset + last[P], last[V], last[A]]) 

    def _evaluate_array(self, t_arr: np.ndarray) -> np.ndarray:
        t_c = np.clip(t_arr, 0.0, self.T)
        N = len(t_c)

        p_out = np.zeros(N)
        v_out = np.zeros(N)
        a_out = np.zeros(N)

        cum_durs = np.cumsum([0.0] + [seg.T for seg in self.segments])
        indices = np.clip(
            np.searchsorted(cum_durs, t_c, side='right') - 1,
            0, len(self.segments) - 1,
        )

        for seg_idx in np.unique(indices):
            mask = indices == seg_idx
            t_local = t_c[mask] - cum_durs[seg_idx]

            seg : TrajectoryProfile = self.segments[seg_idx]
            local = seg.eval(t_local)  # shape (3, M)

            prev_offset = (
                sum(s.eval(s.T)[P] for s in self.segments[:seg_idx])
                if seg_idx > 0 else 0.0
            )

            p_out[mask] = prev_offset + local[P]
            v_out[mask] = local[V]
            a_out[mask] = local[A]

        return np.array([p_out, v_out, a_out])  # shape (3, N)

    def get_duration(self) -> float:
        return self.T

    def check_bounds(self, bounds: np.ndarray, num_samples: int = 100) -> bool:
        return all(seg.check_bounds(bounds, num_samples) for seg in self.segments)

    def find_time_at_position(self, p_target: float, tol: float = 1e-9) -> float | None:
        """Find time t where cumulative position = p_target."""
        p_offset = 0.0
        t_offset = 0.0
        for seg in self.segments:
            p_end_seg = seg.eval(seg.T)[P]
            if p_target <= p_offset + p_end_seg + tol:
                local_t = seg.find_time_at_position(p_target - p_offset, tol)
                if local_t is not None:
                    return t_offset + local_t
            p_offset += p_end_seg
            t_offset += seg.T
        return None