"""CMA-ES inner solver for whole-line design optimization (Track A).

Optimizes the 8-dimensional continuous decision vector theta_c for a fixed N^B,
with multi-restart and best-across-restarts selection.
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field

import numpy as np
import cma

from ksb.optimization.loss import compute_loss, LossResult

log = logging.getLogger(__name__)


# Default box bounds for theta_c (lower, upper).
# Keys match the names used in _theta_to_cfg().
_DEFAULT_BOUNDS: dict[str, tuple[float, float]] = {
    "L_buffer":    (None, 8.0),    # lower set per N^B at solve time
    "beta":        (-2.0, 2.0),
    "gamma":       (-2.0, 2.0),
    "v_buff_out":  (0.5, None),    # upper = Vmax, set at solve time
    "a_u_max":     (0.2, None),    # upper = Amax, set at solve time
    "v_u_max":     (None, None),   # lower = vu_nominal, upper = Vmax, set at solve time
    "eta_s":       (1.0, 2.0),
    "eta_r":       (1.0, 2.0),
}

_THETA_KEYS = ["L_buffer", "beta", "gamma", "v_buff_out", "a_u_max", "v_u_max", "eta_s", "eta_r"]


@dataclass
class InnerResult:
    n_buffer_seg: int
    theta_star: dict             # named optimal params
    L_star: float                # best loss across restarts
    phi_sum_star: float
    U_sum_star: float
    L_buffer_star: float
    eta_r_star: float
    sentinel: bool
    converged: bool              # any restart hit tolfun/tolx (not iter cap)
    n_evals: int                 # total simulation calls across all restarts
    traces: list[np.ndarray]     # per-restart best-so-far per generation


def _build_bounds(n_buffer_seg: int, base_cfg: dict, user_bounds: dict | None) -> dict:
    """Merge default bounds with per-N^B constraints and user overrides."""
    Vmax = float(base_cfg.get("Vmax", 3.0))
    Amax = float(base_cfg.get("Amax", 8.5))
    input_length = float(base_cfg.get("input_length", 0.32))
    Lmin_factor = float(base_cfg.get("Lmin_factor", 1.25))
    Lmin = Lmin_factor * input_length

    arrival_rate_ppm = float(base_cfg.get("arrival_rate_ppm", 180.0))
    gap_mean = float(base_cfg.get("input_gap_mean", 0.6))
    vu_nominal = arrival_rate_ppm / 60.0 * gap_mean

    bounds = {
        "L_buffer":   (n_buffer_seg * Lmin, 8.0),
        "beta":       (-2.0, 2.0),
        "gamma":      (-2.0, 2.0),
        "v_buff_out": (0.5, Vmax),
        "a_u_max":    (0.2, Amax),
        "v_u_max":    (vu_nominal, Vmax),
        "eta_s":      (1.0, 2.0),
        "eta_r":      (1.0, 2.0),
    }

    if user_bounds:
        for k, v in user_bounds.items():
            if k in bounds:
                lo, hi = bounds[k]
                u_lo, u_hi = v
                bounds[k] = (
                    u_lo if u_lo is not None else lo,
                    u_hi if u_hi is not None else hi,
                )

    return bounds


def _theta_to_cfg(theta: np.ndarray, base_cfg: dict, n_buffer_seg: int) -> dict:
    """Map raw decision vector to a cfg dict suitable for KSBSimulation."""
    cfg = dict(base_cfg)
    cfg["n_buffer_seg"] = n_buffer_seg

    (L_buffer, beta, gamma, v_buff_out, a_u_max, v_u_max, eta_s, eta_r) = theta

    input_length = float(base_cfg.get("input_length", 0.32))
    arrival_rate_ppm = float(base_cfg.get("arrival_rate_ppm", 180.0))

    cfg["L_buffer"]        = float(L_buffer)
    cfg["beta"]            = float(beta)
    cfg["gamma"]           = float(gamma)
    cfg["v_buff_out"]      = float(v_buff_out)
    cfg["a_u_max"]         = float(a_u_max)
    cfg["v_u_max"]         = float(v_u_max)
    cfg["slot_length"]     = float(eta_s) * input_length
    cfg["slot_rate_ppm"]   = float(eta_r) * arrival_rate_ppm
    cfg["j_u_max"]         = float(base_cfg.get("jmax", 100.0))   # shared bound

    return cfg


def _theta_from_cfg(base_cfg: dict, n_buffer_seg: int, bounds: dict) -> np.ndarray:
    """Extract a feasible initial point from base_cfg (or midpoint of bounds)."""
    input_length = float(base_cfg.get("input_length", 0.32))
    arrival_rate_ppm = float(base_cfg.get("arrival_rate_ppm", 180.0))

    def _mid(key: str, fallback: float) -> float:
        lo, hi = bounds[key]
        if lo is None and hi is None:
            return fallback
        if lo is None:
            return hi - 0.1
        if hi is None:
            return lo + 0.1
        return (lo + hi) / 2.0

    L_buffer_raw = float(base_cfg.get("L_buffer", 3.0))
    lo_L, hi_L = bounds["L_buffer"]
    L_buffer_clamped = float(np.clip(L_buffer_raw, lo_L, hi_L))

    slot_length = float(base_cfg.get("slot_length", 0.6))
    eta_s_raw = slot_length / input_length
    lo_s, hi_s = bounds["eta_s"]
    eta_s = float(np.clip(eta_s_raw, lo_s, hi_s))

    slot_rate_ppm = float(base_cfg.get("slot_rate_ppm", 216.0))
    eta_r_raw = slot_rate_ppm / arrival_rate_ppm
    lo_r, hi_r = bounds["eta_r"]
    eta_r = float(np.clip(eta_r_raw, lo_r, hi_r))

    return np.array([
        L_buffer_clamped,
        float(base_cfg.get("beta", 0.0)),
        float(base_cfg.get("gamma", 0.0)),
        float(base_cfg.get("v_buff_out", 2.0)),
        float(base_cfg.get("a_u_max", 2.0)),
        float(base_cfg.get("v_u_max", 2.0)),
        eta_s,
        eta_r,
    ])


def solve_inner(
    n_buffer_seg: int,
    base_cfg: dict,
    *,
    lambda_U: float,
    lambda_L: float,
    lambda_T: float,
    bounds: dict[str, tuple[float, float]] | None = None,
    popsize: int = 20,
    max_iter: int = 100,
    n_restarts: int = 4,
    n_seeds_loss: int = 4,
    seed: int = 0,
) -> InnerResult:
    """CMA-ES on theta_c for fixed N^B. Multi-restart, returns best across restarts."""
    resolved_bounds = _build_bounds(n_buffer_seg, base_cfg, bounds)

    lower = np.array([resolved_bounds[k][0] for k in _THETA_KEYS])
    upper = np.array([resolved_bounds[k][1] for k in _THETA_KEYS])
    sigma0 = float(np.mean((upper - lower) / 4.0))

    loss_seeds = list(range(n_seeds_loss))

    t_inner_start = time.perf_counter()
    log.info("NB=%d  starting solve_inner  (popsize=%d  max_iter=%d  restarts=%d  seeds=%d)",
             n_buffer_seg, popsize, max_iter, n_restarts, n_seeds_loss)

    _eval_count = [0]
    _eval_t_accum = [0.0]

    def objective(theta: np.ndarray) -> float:
        theta_clipped = np.clip(theta, lower, upper)
        cfg = _theta_to_cfg(theta_clipped, base_cfg, n_buffer_seg)
        t0 = time.perf_counter()
        try:
            lr = compute_loss(
                cfg,
                lambda_U=lambda_U,
                lambda_L=lambda_L,
                lambda_T=lambda_T,
                seeds=loss_seeds,
            )
            result = lr.L
        except Exception as exc:
            log.debug("objective raised %s: %s", type(exc).__name__, exc)
            result = float("inf")
        dt = time.perf_counter() - t0
        _eval_count[0] += 1
        _eval_t_accum[0] += dt
        return result

    best_L = float("inf")
    best_theta = _theta_from_cfg(base_cfg, n_buffer_seg, resolved_bounds)
    best_loss_result: LossResult | None = None
    converged = False
    total_evals = 0
    traces: list[np.ndarray] = []

    for restart in range(n_restarts):
        rng = np.random.RandomState(seed + restart)
        if restart == 0:
            x0 = _theta_from_cfg(base_cfg, n_buffer_seg, resolved_bounds)
        else:
            x0 = rng.uniform(lower, upper)

        opts = cma.CMAOptions()
        opts["bounds"] = [lower.tolist(), upper.tolist()]
        opts["popsize"] = popsize
        opts["maxiter"] = max_iter
        opts["tolfun"] = 1e-4
        opts["tolx"] = 1e-3
        opts["verbose"] = -9
        opts["seed"] = int(seed + restart)

        log.info("NB=%d  restart %d/%d  starting  (x0 from %s)",
                 n_buffer_seg, restart + 1, n_restarts,
                 "cfg" if restart == 0 else "random")

        es = cma.CMAEvolutionStrategy(x0.tolist(), sigma0, opts)
        trace: list[float] = []
        restart_best = float("inf")
        t_restart_start = time.perf_counter()
        gen = 0

        _eval_count[0] = 0
        _eval_t_accum[0] = 0.0

        while not es.stop():
            t_gen_start = time.perf_counter()
            solutions = es.ask()
            fitnesses = [objective(np.array(s)) for s in solutions]
            total_evals += len(solutions)
            es.tell(solutions, fitnesses)

            gen += 1
            gen_best = float(min(fitnesses))
            restart_best = min(restart_best, gen_best)
            trace.append(restart_best)

            t_gen = time.perf_counter() - t_gen_start
            t_elapsed = time.perf_counter() - t_restart_start
            avg_eval_t = _eval_t_accum[0] / max(_eval_count[0], 1)
            gens_remaining = max_iter - gen
            eta_s = gens_remaining * t_gen

            log.info(
                "NB=%d  restart %d/%d  gen %3d/%d  gen_best=%.4f  restart_best=%.4f"
                "  t_gen=%.1fs  avg_eval=%.3fs  elapsed=%.0fs  eta≈%.0fs",
                n_buffer_seg, restart + 1, n_restarts, gen, max_iter,
                gen_best, restart_best,
                t_gen, avg_eval_t, t_elapsed, eta_s,
            )

        traces.append(np.array(trace))

        stop_reasons = es.stop()
        t_restart = time.perf_counter() - t_restart_start
        log.info(
            "NB=%d  restart %d/%d  done  gens=%d  stop=%s  restart_best=%.4f"
            "  evals=%d  elapsed=%.0fs",
            n_buffer_seg, restart + 1, n_restarts,
            gen, list(stop_reasons.keys()), restart_best,
            _eval_count[0], t_restart,
        )

        hit_tolerance = "tolfun" in stop_reasons or "tolx" in stop_reasons
        if hit_tolerance:
            converged = True

        result_x = np.clip(np.array(es.result.xbest), lower, upper)
        result_L = float(es.result.fbest)

        if result_L < best_L:
            best_L = result_L
            best_theta = result_x

    # Evaluate best_theta to get full loss breakdown
    log.info("NB=%d  all restarts done  total_evals=%d  best_L=%.4f  elapsed=%.0fs",
             n_buffer_seg, total_evals, best_L,
             time.perf_counter() - t_inner_start)

    best_cfg = _theta_to_cfg(best_theta, base_cfg, n_buffer_seg)
    try:
        final_lr = compute_loss(
            best_cfg,
            lambda_U=lambda_U,
            lambda_L=lambda_L,
            lambda_T=lambda_T,
            seeds=loss_seeds,
        )
    except Exception:
        final_lr = LossResult(
            L=best_L, phi_sum=float("nan"), U_sum=float("nan"),
            L_buffer=float(best_theta[0]), eta_r=float(best_theta[7]),
            sentinel=True, per_seed=[],
        )

    theta_star = {k: float(v) for k, v in zip(_THETA_KEYS, best_theta)}
    # Reconstruct derived quantities for reporting
    input_length = float(base_cfg.get("input_length", 0.32))
    arrival_rate_ppm = float(base_cfg.get("arrival_rate_ppm", 180.0))
    theta_star["slot_length"] = theta_star["eta_s"] * input_length
    theta_star["slot_rate_ppm"] = theta_star["eta_r"] * arrival_rate_ppm

    return InnerResult(
        n_buffer_seg=n_buffer_seg,
        theta_star=theta_star,
        L_star=final_lr.L,
        phi_sum_star=final_lr.phi_sum,
        U_sum_star=final_lr.U_sum,
        L_buffer_star=final_lr.L_buffer,
        eta_r_star=final_lr.eta_r,
        sentinel=final_lr.sentinel,
        converged=converged,
        n_evals=total_evals,
        traces=traces,
    )
