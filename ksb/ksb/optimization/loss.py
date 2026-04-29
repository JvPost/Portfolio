"""Loss function for whole-line design optimization (Track A).

Loss: L = sum(Phi) + lambda_U * sum(U) + lambda_L * L_buffer + lambda_T * eta_r
where Phi = (max(0, C - W))^2 (infeasibility barrier) and U = max(0, S) (utilization prior).
lambda_N * N_B is added at outer selection time, not here.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

import numpy as np

from ksb.analysis.cost import compute_Phi_bb, compute_S_bb
from ksb.simulation.ksb_simulation import KSBSimulation


@dataclass
class LossResult:
    L: float                  # total loss
    phi_sum: float            # sum of Phi over all (i, k) cells, averaged across seeds
    U_sum: float              # sum of U over all (i, k) cells, averaged across seeds
    L_buffer: float           # buffer length from cfg
    eta_r: float              # slot_rate_ppm / arrival_rate_ppm
    sentinel: bool            # True if any Phi cell is +inf or NaN
    per_seed: list[dict]      # per-seed diagnostics


def compute_loss(
    cfg: dict,
    *,
    lambda_U: float,
    lambda_L: float,
    lambda_T: float,
    n_seeds: int = 1,
    seeds: Sequence[int] | None = None,
) -> LossResult:
    """Run KSBSimulation for one or more seeds and return loss components.

    Sentinel handling: if any Phi cell is +inf or NaN across any seed,
    sentinel=True and L=+inf. CMA-ES treats +inf as infeasible.
    """
    if seeds is None:
        seeds = list(range(n_seeds))

    j_max = float(cfg["jmax"])
    L_buffer = float(cfg.get("L_buffer", 2.0))
    eta_r = float(cfg.get("eta_r", 1.0))

    phi_accum = 0.0
    U_accum = 0.0
    sentinel = False
    per_seed: list[dict] = []

    for seed in seeds:
        try:
            result = KSBSimulation(cfg=cfg).run(seed=seed, skip_pair_records=True)
        except Exception as e:
            sentinel = True
            per_seed.append({"seed": seed, "error": str(e)})
            continue

        events = result.segment_events
        if events is None:
            sentinel = True
            per_seed.append({"seed": seed, "error": "segment_events is None (batch < 2)"})
            continue

        Phi = compute_Phi_bb(events, j_max)     # (b-1, N_B)
        S = compute_S_bb(events, j_max)         # (b-1, N_B)
        U = np.maximum(0.0, S)                  # utilization

        phi_sum_seed = float(np.sum(Phi))
        U_sum_seed = float(np.sum(U))

        if not np.isfinite(phi_sum_seed):
            sentinel = True

        per_seed.append({
            "seed": seed,
            "phi_sum": phi_sum_seed,
            "U_sum": U_sum_seed,
            "sentinel": not np.isfinite(phi_sum_seed),
        })

        phi_accum += phi_sum_seed
        U_accum += U_sum_seed

    n = len(seeds)
    phi_mean = phi_accum / n
    U_mean = U_accum / n

    if sentinel:
        L = float("inf")
    else:
        L = phi_mean + lambda_U * U_mean + lambda_L * L_buffer + lambda_T * eta_r

    return LossResult(
        L=L,
        phi_sum=phi_mean,
        U_sum=U_mean,
        L_buffer=L_buffer,
        eta_r=eta_r,
        sentinel=sentinel,
        per_seed=per_seed,
    )
