from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ksb.analysis.events import SegmentEvents


@dataclass
class BBCostResult:
    """Per-(pair, segment) bang-bang minimum transition time and diagnostics.

    See docs/Buffer minimum-time cost.md.
    """
    C: np.ndarray          # (b-1, N_B) minimum transition time; +inf where infeasible
    a_peak: np.ndarray     # (b-1, N_B) signed peak/trough acceleration of chosen case
    case: np.ndarray       # (b-1, N_B) int8: +1 up, -1 down, 0 infeasible
    primitive_exists: np.ndarray   # (b-1, N_B) bool
        # Whether a jerk-bounded bang-bang profile exists between the endpoint
        # kinematic states (v-, a-) -> (v+, a+). Note: this is NOT the same as
        # "feasible in the design objective sense" — that requires C <= W
        # (the primitive fits in the available time window). For budget-feasibility,
        # compare C against events.W or use compute_Phi_bb.


def compute_C_bb(events: SegmentEvents, j_max: float) -> BBCostResult:
    a_m = events.a_minus
    a_p = events.a_plus
    dv = events.v_plus - events.v_minus

    half_sum_sq = 0.5 * (a_m ** 2 + a_p ** 2)

    disc_up = j_max * dv + half_sum_sq
    a_peak_up = np.sqrt(np.maximum(disc_up, 0.0))
    C_up = (2.0 * a_peak_up - a_m - a_p) / j_max
    feasible_up = (disc_up >= 0.0) & (a_peak_up >= np.maximum(a_m, a_p))

    disc_down = -j_max * dv + half_sum_sq
    a_peak_down = -np.sqrt(np.maximum(disc_down, 0.0))
    C_down = (a_m + a_p - 2.0 * a_peak_down) / j_max
    feasible_down = (disc_down >= 0.0) & (a_peak_down <= np.minimum(a_m, a_p))

    C_up_masked = np.where(feasible_up, C_up, np.inf)
    C_down_masked = np.where(feasible_down, C_down, np.inf)

    C = np.minimum(C_up_masked, C_down_masked)

    assert not any(C == np.inf)

    up_wins = feasible_up & (C_up_masked <= C_down_masked)
    down_wins = feasible_down & ~up_wins

    case = np.zeros(C.shape, dtype=np.int8)
    case[up_wins] = 1
    case[down_wins] = -1

    a_peak = np.where(up_wins, a_peak_up, np.where(down_wins, a_peak_down, 0.0))
    primitive_exists = feasible_up | feasible_down

    return BBCostResult(C=C, a_peak=a_peak, case=case, primitive_exists=primitive_exists)


def compute_S_bb(events: SegmentEvents, j_max: float) -> np.ndarray:
    """Slack matrix W - C for bang-bang primitive."""
    cost: BBCostResult = compute_C_bb(events, j_max) 
    return events.W - cost.C


def compute_Phi_bb(events: SegmentEvents, j_max: float) -> np.ndarray:
    """Infeasibility penalty (max(0, C - W))^2 for bang-bang primitive."""
    S = compute_S_bb(events, j_max)
    return np.maximum(0.0, -S) ** 2
