"""CMA-ES inner solver for whole-line design optimization (Track A).

Optimizes the 4-dimensional continuous decision vector theta_c for a fixed N^B,
with multi-restart and best-across-restarts selection.
"""
from __future__ import annotations

import logging
import time
from collections import Counter
from dataclasses import dataclass, field

import numpy as np
import cma

from ksb.optimization.loss import compute_loss, LossResult

log = logging.getLogger(__name__)


# Default box bounds for theta_c (lower, upper).
# Keys match the names used in _theta_to_cfg().
_DEFAULT_BOUNDS: dict[str, tuple[float, float]] = {
    "L_buffer":    (None, 8.0),    # lower set per N^B at solve time
    "eta_r":       (1.0, 2.0),
    "eta_s":       (1.0, 2.0),
    "eta_v":       (1.0, 2.0),
}

_THETA_KEYS = ["L_buffer", "eta_r", "eta_s", "eta_v"]


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
    input_length = float(base_cfg.get("input_length", 0.32))
    Lmin_factor = float(base_cfg.get("Lmin_factor", 1.25))
    Lmin = Lmin_factor * input_length

    bounds = {
        "L_buffer":   (n_buffer_seg * Lmin, 8.0),
        "eta_r":      (1.0, 2.0),
        "eta_s":      (1.0, 2.0),
        "eta_v":      (1.0, 2.0),
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

    (L_buffer, eta_r, eta_s, eta_v) = theta

    cfg["L_buffer"] = float(L_buffer)
    cfg["eta_r"]    = float(eta_r)
    cfg["eta_s"]    = float(eta_s)
    cfg["eta_v"]    = float(eta_v)

    return cfg


def _to_unit(x_phys: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    """Map physical-unit theta to the unit cube [0, 1]^d."""
    return (x_phys - lower) / (upper - lower)


def _to_physical(x_unit: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    """Map unit-cube point back to physical units."""
    return lower + x_unit * (upper - lower)


def _theta_from_cfg(base_cfg: dict, n_buffer_seg: int, bounds: dict) -> np.ndarray:
    """Extract a feasible initial point from base_cfg (or midpoint of bounds)."""
    L_buffer_raw = float(base_cfg.get("L_buffer", 3.0))
    lo_L, hi_L = bounds["L_buffer"]
    L_buffer_clamped = float(np.clip(L_buffer_raw, lo_L, hi_L))

    lo_r, hi_r = bounds["eta_r"]
    eta_r = float(np.clip(base_cfg.get("eta_r", 1.2), lo_r, hi_r))

    lo_s, hi_s = bounds["eta_s"]
    eta_s = float(np.clip(base_cfg.get("eta_s", 1.5), lo_s, hi_s))

    lo_v, hi_v = bounds["eta_v"]
    eta_v = float(np.clip(base_cfg.get("eta_v", 1.16), lo_v, hi_v))

    return np.array([L_buffer_clamped, eta_r, eta_s, eta_v])


def _project_to_feasibility(
    theta_phys: np.ndarray, V_max: float, r_u: float, l: float
) -> tuple[np.ndarray, str]:
    """Cascading projection onto the V_max-feasible tetrahedron.

    Constraint: eta_v * eta_r * eta_s <= V_max / (r_u * l).
    Cascade order follows asymmetric commitment of the eta-coordinates: eta_s is
    clipped before eta_v because eta_v is the least committed (unpriced; only
    constrained by feasibility) and eta_s is less committed than eta_r (which is
    priced explicitly via lambda_T * eta_r in the loss). See "Constraint geometry
    and enforcement" in Optimization notes.md.

    Box bounds are NOT re-enforced. On the configured hardware the cascade is
    feasible w.r.t. boxes by construction; the assertion in ksb_simulation.py is
    the canary if the hardware envelope changes.

    Returns (theta_projected, event) where event is one of:
        "none"   — no clip needed
        "step2"  — only eta_v clipped (typical case when wall binds)
        "both"   — eta_s clipped (wedge case) and eta_v subsequently clipped
    Step 1 alone ("step1") is impossible: clipping eta_s sets eta_r*eta_s exactly
    to vmax_budget, so any eta_v > 1 triggers step 2.
    """
    theta = theta_phys.copy()
    # _THETA_KEYS = ["L_buffer", "eta_r", "eta_s", "eta_v"]
    eta_r, eta_s, eta_v = theta[1], theta[2], theta[3]
    event = "none"

    # Mirrors the simulation's exact multiplication order (ksb_simulation.py:59,63):
    #   rd = eta_r * ru;  slot_length = eta_s * l;  vd = rd * slot_length
    # Using the same float ops as the simulation avoids arithmetic-order divergence
    # between the projector and the sim's vd computation.
    vd_sim = (eta_r * r_u) * (eta_s * l)

    # Step 1: project eta_s if (eta_r, eta_s) is in the bad wedge.
    # Condition uses vd_sim (not a separate vmax_budget expression) so the wedge
    # check and the step-2 eta_v_max use the same arithmetic path — preventing the
    # case where a different fp expression misses the wedge and step 2 then
    # produces eta_v_max < 1.
    if vd_sim > V_max:
        eta_s = V_max / ((eta_r * r_u) * l)
        theta[2] = eta_s
        # Recompute and clamp: the clip arithmetic can overshoot V_max by ~1 ULP.
        vd_sim = min((eta_r * r_u) * (eta_s * l), V_max)
        event = "step1"

    # Step 2: project eta_v.
    # vd_sim <= V_max here, so eta_v_max >= 1.
    eta_v_max = V_max / vd_sim
    if eta_v > eta_v_max:
        theta[3] = eta_v_max
        event = "both" if event == "step1" else "step2"

    return theta, event


def solve_inner(
    n_buffer_seg: int,
    base_cfg: dict,
    *,
    lambda_U: float,
    lambda_L: float,
    lambda_T: float,
    lambda_N: float,
    bounds: dict[str, tuple[float, float]] | None = None,
    popsize: int = 20,
    max_iter: int = 100,
    n_restarts: int = 4,
    n_seeds_loss: int = 4,
    seed: int = 0,
) -> InnerResult:
    """CMA-ES on theta_c for fixed N^B. Multi-restart, returns best across restarts.

    Optimizes in the unit cube [0,1]^d with sigma0=0.25 (isotropic).  Physical-unit
    bounds are used only to map to/from unit space; the optimizer sees [0,1] for
    every coordinate.
    """
    resolved_bounds = _build_bounds(n_buffer_seg, base_cfg, bounds)

    lower = np.array([resolved_bounds[k][0] for k in _THETA_KEYS])
    upper = np.array([resolved_bounds[k][1] for k in _THETA_KEYS])
    sigma0 = 0.25   # one quarter of the unit-cube axis; isotropic by construction
    unit_lo = [0.0] * len(_THETA_KEYS)
    unit_hi = [1.0] * len(_THETA_KEYS)

    r_u = float(base_cfg.get("arrival_rate_ppm", 180.0)) / 60.0
    l = float(base_cfg.get("input_length", 0.32))
    V_max = float(base_cfg.get("Vmax", 3.0))

    loss_seeds = list(range(n_seeds_loss))

    t_inner_start = time.perf_counter()
    log.info("NB=%d  starting solve_inner  (popsize=%d  max_iter=%d  restarts=%d  seeds=%d)",
             n_buffer_seg, popsize, max_iter, n_restarts, n_seeds_loss)

    _eval_count = [0]
    _eval_t_accum = [0.0]
    _exc_counts: Counter = Counter()
    _proj_counts: Counter = Counter()

    def objective(x_unit: np.ndarray) -> float:
        theta = _to_physical(np.clip(x_unit, 0.0, 1.0), lower, upper)
        theta, proj_event = _project_to_feasibility(theta, V_max, r_u, l)
        _proj_counts[proj_event] += 1
        cfg = _theta_to_cfg(theta, base_cfg, n_buffer_seg)
        t0 = time.perf_counter()
        try:
            lr = compute_loss(
                cfg,
                lambda_U=lambda_U,
                lambda_L=lambda_L,
                lambda_T=lambda_T,
                lambda_N=lambda_N,
                seeds=loss_seeds,
            )
            result = lr.L
        except Exception as exc:
            exc_type = type(exc).__name__
            _exc_counts[exc_type] += 1
            log.warning("NB=%d  objective exception  %s: %s", n_buffer_seg, exc_type, exc)
            result = float("inf")
        dt = time.perf_counter() - t0
        _eval_count[0] += 1
        _eval_t_accum[0] += dt
        return result

    best_L = float("inf")
    best_theta = _theta_from_cfg(base_cfg, n_buffer_seg, resolved_bounds)
    converged = False
    total_evals = 0
    traces: list[np.ndarray] = []

    for restart in range(n_restarts):
        rng = np.random.RandomState(seed + restart)
        if restart == 0:
            x0_phys = _theta_from_cfg(base_cfg, n_buffer_seg, resolved_bounds)
            x0 = _to_unit(x0_phys, lower, upper)
        else:
            x0 = rng.uniform(0.0, 1.0, size=len(_THETA_KEYS))

        opts = cma.CMAOptions()
        opts["bounds"] = [unit_lo, unit_hi]
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

        best_unit = np.clip(np.array(es.result.xbest), 0.0, 1.0)
        result_x = _to_physical(best_unit, lower, upper)
        result_L = float(es.result.fbest)

        if result_L < best_L:
            best_L = result_L
            best_theta = result_x

    # Evaluate best_theta to get full loss breakdown
    log.info("NB=%d  all restarts done  total_evals=%d  best_L=%.4f  elapsed=%.0fs",
             n_buffer_seg, total_evals, best_L,
             time.perf_counter() - t_inner_start)
    if _exc_counts:
        summary = "  ".join(f"{k}={v}" for k, v in sorted(_exc_counts.items()))
        log.warning("NB=%d  exception summary: %s  (total=%d / %d evals)",
                    n_buffer_seg, summary, sum(_exc_counts.values()), total_evals)
    if _proj_counts:
        summary = "  ".join(f"{k}={v}" for k, v in sorted(_proj_counts.items()))
        log.info("NB=%d  projection summary: %s  (total=%d / %d evals)",
                 n_buffer_seg, summary, sum(_proj_counts.values()), total_evals)

    best_cfg = _theta_to_cfg(best_theta, base_cfg, n_buffer_seg)
    try:
        final_lr = compute_loss(
            best_cfg,
            lambda_U=lambda_U,
            lambda_L=lambda_L,
            lambda_T=lambda_T,
            lambda_N=lambda_N,
            seeds=loss_seeds,
        )
    except Exception:
        final_lr = LossResult(
            L=best_L, phi_sum=float("nan"), U_sum=float("nan"),
            L_buffer=float(best_theta[0]), eta_r=float(best_theta[1]),
            sentinel=True, per_seed=[],
        )

    theta_star = {k: float(v) for k, v in zip(_THETA_KEYS, best_theta)}

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
