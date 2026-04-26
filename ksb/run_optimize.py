#!/usr/bin/env python3
"""Track A: whole-line design optimization via CMA-ES.

Sweeps N^B from N_min to N_max, runs CMA-ES inner solver for each,
saves per-N^B JSON results and an outer summary.

Usage:
    python run_optimize.py [--N-min 3] [--N-max 20] [--lambda-U 0.05]
                           [--lambda-L 0.5] [--lambda-T 1.0] [--lambda-N 0.1]
                           [--popsize 20] [--restarts 4] [--n-seeds 4]
                           [--config default] [--out results/track_a/]
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
import yaml

from ksb.optimization.cma_solver import InnerResult, solve_inner

_CONFIG_DIR = Path(__file__).parent / "configs"


def _result_to_dict(r: InnerResult) -> dict:
    return {
        "n_buffer_seg": r.n_buffer_seg,
        "theta_star": r.theta_star,
        "L_star": r.L_star,
        "phi_sum_star": r.phi_sum_star,
        "U_sum_star": r.U_sum_star,
        "L_buffer_star": r.L_buffer_star,
        "eta_r_star": r.eta_r_star,
        "sentinel": r.sentinel,
        "converged": r.converged,
        "n_evals": r.n_evals,
    }


def main():
    parser = argparse.ArgumentParser(description="Track A: whole-line design optimization")
    parser.add_argument("--N-min", type=int, default=3, help="Minimum N^B (default: 3)")
    parser.add_argument("--N-max", type=int, default=20, help="Maximum N^B (default: 20)")
    parser.add_argument("--lambda-U", type=float, default=0.05, help="Utilization weight (default: 0.05)")
    parser.add_argument("--lambda-L", type=float, default=0.5, help="Buffer length weight (default: 0.5)")
    parser.add_argument("--lambda-T", type=float, default=1.0, help="Throughput ratio weight (default: 1.0)")
    parser.add_argument("--lambda-N", type=float, default=0.1, help="Segment count weight for outer selection (default: 0.1)")
    parser.add_argument("--popsize", type=int, default=20, help="CMA-ES population size (default: 20)")
    parser.add_argument("--restarts", type=int, default=4, help="CMA-ES restarts per N^B (default: 4)")
    parser.add_argument("--n-seeds", type=int, default=4, help="Seeds per loss evaluation (default: 4)")
    parser.add_argument("--config", type=str, default="default", help="Config name in configs/ (default: default)")
    parser.add_argument("--out", type=str, default="results/track_a/", help="Output directory (default: results/track_a/)")
    parser.add_argument("--batch", type=int, default=None,
                        help="Override cfg batch size for optimizer (smaller = faster; default: use config value)")
    parser.add_argument("--max-iter", type=int, default=100,
                        help="CMA-ES max generations per restart (default: 100)")
    args = parser.parse_args()

    with open(_CONFIG_DIR / f"{args.config}.yaml") as f:
        base_cfg = yaml.safe_load(f)

    if args.batch is not None:
        base_cfg["batch"] = args.batch

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    nb_range = range(args.N_min, args.N_max + 1)
    inner_results: dict[int, InnerResult] = {}
    t_wall_start = time.time()

    for nb in nb_range:
        t0 = time.time()
        r = solve_inner(
            n_buffer_seg=nb,
            base_cfg=base_cfg,
            lambda_U=args.lambda_U,
            lambda_L=args.lambda_L,
            lambda_T=args.lambda_T,
            popsize=args.popsize,
            max_iter=args.max_iter,
            n_restarts=args.restarts,
            n_seeds_loss=args.n_seeds,
            seed=0,
        )
        dt = time.time() - t0
        inner_results[nb] = r

        # Save per-N^B result
        result_path = out_dir / f"inner_NB_{nb:02d}.json"
        with open(result_path, "w") as f:
            json.dump(_result_to_dict(r), f, indent=2)

        ts = r.theta_star
        print(
            f"  NB={nb:02d}  L*={r.L_star:.4f}  L_B*={ts['L_buffer']:.2f}"
            f"  v_BR*={ts['v_buff_out']:.2f}  beta*={ts['beta']:.2f}"
            f"  gamma*={ts['gamma']:.2f}  eta_s*={ts['eta_s']:.2f}"
            f"  eta_r*={ts['eta_r']:.2f}  evals={r.n_evals}  conv={r.converged}"
            f"  ({dt:.1f}s)"
        )

    # Outer selection: L*(N^B) + lambda_N * N^B
    outer_scores = {
        nb: inner_results[nb].L_star + args.lambda_N * nb
        for nb in nb_range
    }
    nb_star = min(outer_scores, key=outer_scores.__getitem__)

    total_time = time.time() - t_wall_start
    print(f"\nTotal wall time: {total_time:.1f}s")
    print(f"Optimal N^B* = {nb_star}  (L* + λ_N·N^B = {outer_scores[nb_star]:.4f})")

    outer_summary = {
        "config": args.config,
        "lambda_U": args.lambda_U,
        "lambda_L": args.lambda_L,
        "lambda_T": args.lambda_T,
        "lambda_N": args.lambda_N,
        "N_min": args.N_min,
        "N_max": args.N_max,
        "nb_star": nb_star,
        "outer_scores": {str(nb): outer_scores[nb] for nb in nb_range},
        "inner_results": {str(nb): _result_to_dict(inner_results[nb]) for nb in nb_range},
        "total_wall_time_s": total_time,
    }

    summary_path = out_dir / "outer_summary.json"
    with open(summary_path, "w") as f:
        json.dump(outer_summary, f, indent=2)

    print(f"Results written to {out_dir}/")


if __name__ == "__main__":
    main()
