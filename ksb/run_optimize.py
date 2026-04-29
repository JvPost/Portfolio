#!/usr/bin/env python3
"""Track A: whole-line design optimization via CMA-ES.

Sweeps N^B from N_min to N_max (from optimizer config), runs CMA-ES inner
solver for each, saves per-N^B JSON results and an outer summary.

Output goes to results/<datetime>/ automatically.

Usage:
    python run_optimize.py
    python run_optimize.py --system-config default --opt-config quick
    python run_optimize.py --batch 20   # override batch size for speed
"""
from __future__ import annotations

import argparse
import json
import logging
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml

from ksb.optimization.cma_solver import InnerResult, solve_inner

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-5s  %(message)s",
    datefmt="%H:%M:%S",
)

_CONFIG_DIR = Path(__file__).parent / "configs"
_RESULTS_DIR = Path(__file__).parent / "results"


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
    parser.add_argument("--system-config", type=str, default="default",
                        help="System config name in configs/system/ (default: default)")
    parser.add_argument("--opt-config", type=str, default="default",
                        help="Optimizer config name in configs/optimizer/ (default: default)")
    parser.add_argument("--batch", type=int, default=None,
                        help="Override batch size from system config (smaller = faster)")
    args = parser.parse_args()

    sys_cfg_path = _CONFIG_DIR / "system" / f"{args.system_config}.yaml"
    opt_cfg_path = _CONFIG_DIR / "optimizer" / f"{args.opt_config}.yaml"

    with open(sys_cfg_path) as f:
        base_cfg = yaml.safe_load(f)
    with open(opt_cfg_path) as f:
        opt_cfg = yaml.safe_load(f)

    if args.batch is not None:
        base_cfg["batch"] = args.batch

    # Timestamped output directory
    run_tag = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    out_dir = _RESULTS_DIR / run_tag
    out_dir.mkdir(parents=True, exist_ok=True)

    # File handler — all log output (INFO + warnings) goes to optimize.log
    log_path = out_dir / "optimize.log"
    _file_handler = logging.FileHandler(log_path)
    _file_handler.setLevel(logging.DEBUG)
    _file_handler.setFormatter(logging.Formatter(
        "%(asctime)s  %(levelname)-5s  %(message)s", datefmt="%H:%M:%S"
    ))
    logging.getLogger().addHandler(_file_handler)

    # Snapshot the configs used for this run
    run_info = {
        "run_tag": run_tag,
        "system_config": args.system_config,
        "opt_config": args.opt_config,
        "system_cfg": base_cfg,
        "opt_cfg": opt_cfg,
    }
    with open(out_dir / "run_info.yaml", "w") as f:
        yaml.dump(run_info, f, default_flow_style=False)

    nb_range = range(opt_cfg["N_min"], opt_cfg["N_max"] + 1)
    inner_results: dict[int, InnerResult] = {}
    t_wall_start = time.time()

    nb_list = list(nb_range)
    for nb_idx, nb in enumerate(nb_list):
        t0 = time.time()
        print(f"\n[{nb_idx+1}/{len(nb_list)}] NB={nb}  starting  "
              f"(popsize={opt_cfg['popsize']}, max_iter={opt_cfg['max_iter']}, "
              f"restarts={opt_cfg['restarts']}, n_seeds={opt_cfg['n_seeds']})")
        r = solve_inner(
            n_buffer_seg=nb,
            base_cfg=base_cfg,
            lambda_U=opt_cfg["lambda_U"],
            lambda_L=opt_cfg["lambda_L"],
            lambda_T=opt_cfg["lambda_T"],
            popsize=opt_cfg["popsize"],
            max_iter=opt_cfg["max_iter"],
            n_restarts=opt_cfg["restarts"],
            n_seeds_loss=opt_cfg["n_seeds"],
            seed=0,
        )
        dt = time.time() - t0
        inner_results[nb] = r

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
        nb: inner_results[nb].L_star + opt_cfg["lambda_N"] * nb
        for nb in nb_range
    }
    nb_star = min(outer_scores, key=outer_scores.__getitem__)

    total_time = time.time() - t_wall_start
    print(f"\nTotal wall time: {total_time:.1f}s")
    print(f"Optimal N^B* = {nb_star}  (L* + λ_N·N^B = {outer_scores[nb_star]:.4f})")

    outer_summary = {
        "run_tag": run_tag,
        "system_config": args.system_config,
        "opt_config": args.opt_config,
        "opt_cfg": opt_cfg,
        "nb_star": nb_star,
        "outer_scores": {str(nb): outer_scores[nb] for nb in nb_range},
        "inner_results": {str(nb): _result_to_dict(inner_results[nb]) for nb in nb_range},
        "total_wall_time_s": total_time,
    }

    with open(out_dir / "outer_summary.json", "w") as f:
        json.dump(outer_summary, f, indent=2)

    print(f"Results written to {out_dir}/")


if __name__ == "__main__":
    main()
