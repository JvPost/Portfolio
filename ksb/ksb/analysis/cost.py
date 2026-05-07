from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ksb.analysis.events import SegmentEvents


@dataclass
class BBCostResult:
    """Per-(pair, segment) bang-bang minimum transition time and diagnostics.

    See docs/Buffer minimum-time cost.md §4 and §6.
    """
    C: np.ndarray           # (b-1, N_B) minimum transition time; finite everywhere (adaptive jerk)
    a_peak: np.ndarray      # (b-1, N_B) signed peak/trough acceleration of chosen case
    case: np.ndarray        # (b-1, N_B) int8: +1 up, -1 down
    j_req_active: np.ndarray  # (b-1, N_B) bool: True where j_req > j_max (adaptive jerk was needed)
        # Cells where j_req_active is True required j̃ > j^B_max to render the primitive feasible.
        # Under the KSB operating envelope these should be empty; non-empty cells are diagnostic.


def compute_j_req(events: SegmentEvents) -> np.ndarray:
    """Minimum jerk for a feasible bang-bang primitive, per (pair, segment) cell.

    Returns j_req = max{0, (M² - m²) / (2Δv)} where M = max(a⁻, a⁺),
    m = min(a⁻, a⁺), Δv = v⁺ - v⁻ signed.  See docs/Buffer minimum-time cost.md §6.1.

    At Δv = 0 at least one primitive is always feasible at any j > 0, so j_req = 0.
    """
    M = np.maximum(events.a_minus, events.a_plus)
    m = np.minimum(events.a_minus, events.a_plus)
    dv = events.v_plus - events.v_minus
    numer = M ** 2 - m ** 2
    return np.where(dv == 0.0, 0.0, np.maximum(0.0, numer / (2.0 * dv)))


def compute_C_bb(events: SegmentEvents, j_max: float) -> BBCostResult:
    """Bang-bang minimum transition time with adaptive jerk extension.

    Uses j̃ = max(j_max, j_req) per cell so that C is finite everywhere.
    See docs/Buffer minimum-time cost.md §6.2.
    """
    j_req = compute_j_req(events)
    j_tilde = np.maximum(j_max, j_req * (1.0 + 1e-9))

    a_m = events.a_minus
    a_p = events.a_plus
    dv = events.v_plus - events.v_minus

    half_sum_sq = 0.5 * (a_m ** 2 + a_p ** 2)

    disc_up = j_tilde * dv + half_sum_sq
    a_peak_up = np.sqrt(np.maximum(disc_up, 0.0))
    C_up = (2.0 * a_peak_up - a_m - a_p) / j_tilde
    feasible_up = (disc_up >= 0.0) & (a_peak_up >= np.maximum(a_m, a_p))

    disc_down = -j_tilde * dv + half_sum_sq
    a_peak_down = -np.sqrt(np.maximum(disc_down, 0.0))
    C_down = (a_m + a_p - 2.0 * a_peak_down) / j_tilde
    feasible_down = (disc_down >= 0.0) & (a_peak_down <= np.minimum(a_m, a_p))

    C_up_masked = np.where(feasible_up, C_up, np.inf)
    C_down_masked = np.where(feasible_down, C_down, np.inf)

    C = np.minimum(C_up_masked, C_down_masked)

    up_wins = feasible_up & (C_up_masked <= C_down_masked)
    down_wins = feasible_down & ~up_wins

    case = np.where(up_wins, np.int8(1), np.where(down_wins, np.int8(-1), np.int8(0)))
    a_peak = np.where(up_wins, a_peak_up, np.where(down_wins, a_peak_down, 0.0))
    j_req_active = j_req > j_max

    return BBCostResult(C=C, a_peak=a_peak, case=case, j_req_active=j_req_active)


def compute_delta_j(events: SegmentEvents, j_max: float) -> np.ndarray:
    """Per-cell jerk overspec: max{0, j_req - j_max}.

    Post-hoc diagnostic only — not used in the loss.  See docs/Buffer minimum-time cost.md §6.3.
    """
    return np.maximum(0.0, compute_j_req(events) - j_max)


def compute_S_bb(events: SegmentEvents, j_max: float) -> np.ndarray:
    """Slack matrix W - C for bang-bang primitive (adaptive jerk)."""
    return events.W - compute_C_bb(events, j_max).C


def compute_Phi_bb(events: SegmentEvents, j_max: float) -> np.ndarray:
    """One-sided infeasibility barrier (max(0, C - W))² for bang-bang primitive.

    Diagnostic function matching spec §4.1.  Used in the loss; also useful for
    per-cell visualization of where infeasibility is concentrated.
    """
    S = compute_S_bb(events, j_max)
    return np.maximum(0.0, -S) ** 2
