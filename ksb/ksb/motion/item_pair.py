from dataclasses import dataclass
from typing import List, Optional

import numpy as np
from scipy.integrate import simpson, trapezoid

from ksb.motion.trajectories import TrajectoryProfile, P, V, A


@dataclass
class PairRecord:
    """
    Container for the gap curve between a consecutive item pair, evaluated
    over the overlap window where both items are simultaneously in the buffer.

    gap is defined as:
        g(t) = p_lead(t + delta_t) - p_follow(t)

    All integrals are computed relative to p_min, so that zero is the constraint
    boundary.
    """
    pair_index: int
    lead: TrajectoryProfile
    follow: TrajectoryProfile

    t: np.ndarray        # time points in follower's local frame
    gap: np.ndarray      # g(t) = p_lead(t + delta_t) - p_follow(t)
    delta_v: np.ndarray  # Δv(t) = v_lead(t + delta_t) - v_follow(t)
    delta_a: np.ndarray  # Δa(t) = a_lead(t + delta_t) - a_follow(t)

    delta_t: float       # arrival delay of follower relative to leader
    t_start: float       # start of evaluation window (follower local time)
    t_end: float         # end of evaluation window (follower local time)

    duration: float = 0.
    g_min_threshold: float = 0.

    margin_integral: Optional[float] = None
    average_margin: Optional[float] = None
    gap_integral: Optional[float] = None       
    average_gap_integral: Optional[float] = None
    min_gap: Optional[float] = None
    violation_integral: Optional[float] = None
    violation_duration: Optional[float] = None

    def __post_init__(self):
        self.duration = self.t_end - self.t_start

    def compute_integrals(self, g_min: float = 0., method: str = 'simpson') -> None:
        """Compute all gap integrals relative to p_min as the baseline."""
        if len(self.gap) == 0:
            return

        self.g_min_threshold = g_min
        integrate = simpson if method == 'simpson' else trapezoid

        margin = self.gap - g_min

        self.margin_integral = integrate(margin, self.t)

        if self.duration > 1e-9:
            self.average_margin = self.margin_integral / self.duration
        else:
            self.average_margin = np.nan

        self.gap_integral = self.margin_integral + self.g_min_threshold
        self.average_gap_integral = self.average_margin + self.g_min_threshold

        self.min_gap = float(np.min(self.gap))

        if g_min > 0.:
            tol_v = 1e-6
            tol_a = 1e-6
            lockstep = (np.abs(self.delta_v) < tol_v) & (np.abs(self.delta_a) < tol_a)

            below = np.maximum(-margin, 0.0)
            below_masked = np.where(lockstep, 0.0, below)
            self.violation_integral = integrate(below_masked, self.t)

            is_below = ((self.gap < g_min) & ~lockstep).astype(float)
            self.violation_duration = integrate(is_below, self.t)
        else:
            self.violation_integral = self.margin_integral
            self.violation_duration = self.duration


def compute_pairs(
    trajectories: List[TrajectoryProfile],
    delta_t: np.ndarray,                        # shape (n-1,)
    t_rel_start: Optional[np.ndarray] = None,   # shape (n-1,) or None → default 0
    t_rel_end: Optional[np.ndarray] = None,     # shape (n-1,) or None → default follow.T
    n_points: int = 1000,
) -> List[PairRecord]:
    """
    Compute pair objects for each consecutive pair of trajectories.

    - delta_t[i] = how much later follower i+1 starts compared to leader i
    - t_rel_start / t_rel_end are in follower's local time
    - If None → full overlap [0, follow.T]
    """
    n_pairs = len(trajectories) - 1
    if len(delta_t) != n_pairs:
        raise ValueError("delta_t must have length n_trajectories - 1")

    if t_rel_start is None:
        t_rel_start = np.zeros(n_pairs)

    if t_rel_end is None:
        t_rel_end = np.array([traj.T for traj in trajectories[1:]])

    if len(t_rel_start) != len(t_rel_end) != n_pairs:
        raise ValueError("t_rel_start and t_rel_end must have length n_pairs")

    pairs: List[PairRecord] = []

    for i in range(n_pairs):
        lead = trajectories[i]
        follow = trajectories[i + 1]
        dt = delta_t[i]

        start = t_rel_start[i]
        end = t_rel_end[i]

        if start < 0 or end <= start or end > follow.T:
            raise ValueError(
                f"Invalid window for pair {i}: start={start}, end={end}, follow.T={follow.T}"
            )

        t_follow = np.linspace(start, end, n_points)
        t_lead = t_follow + dt

        if np.any(t_lead > lead.T):
            # raise ValueError(
            #     f"Leader extrapolation needed for pair {i} — "
            #     f"max t_lead = {t_lead.max():.4f} > lead.T = {lead.T:.4f}. "
            # )
            print(f"Time error at gap {i+1}")

        s_follow = follow.eval(t_follow)  # shape (3, N)
        s_lead = lead.eval(t_lead)         # shape (3, N)

        p = s_lead[P] - s_follow[P]
        delta_v = s_lead[V] - s_follow[V]
        delta_a = s_lead[A] - s_follow[A]

        pair = PairRecord(
            pair_index=i,
            lead=lead,
            follow=follow,
            t=t_follow,
            gap=p,
            delta_v=delta_v,
            delta_a=delta_a,
            delta_t=dt,
            t_start=start,
            t_end=end,
        )
        pairs.append(pair)

    return pairs
