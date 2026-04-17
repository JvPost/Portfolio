#!/usr/bin/env python3
"""Run a KSB simulation and print a summary to stdout.

Usage:
    python run_sim.py [--seed INT] [--config NAME]
"""
import argparse
from pathlib import Path

import numpy as np
import yaml

from ksb.planning.solvers.quintic import QuinticSolver
from ksb.planning.solvers.scurve import SCurveSolver
from ksb.simulation.ksb_simulation import KSBSimulation

_CONFIG_DIR = Path(__file__).parent / "configs"


def main():
    parser = argparse.ArgumentParser(description="KSB simulation runner")
    parser.add_argument("--seed", type=int, default=42,
                        help="Random seed for arrival times (default: 42)")
    parser.add_argument("--config", type=str, default="default",
                        help="Config file in configs/ without .yaml suffix (default: default)")
    args = parser.parse_args()

    with open(_CONFIG_DIR / f"{args.config}.yaml") as f:
        cfg = yaml.safe_load(f)

    solver_name = cfg.get("solver", "scurve")
    print(f"Solver : {solver_name}")
    print(f"Seed   : {args.seed}")
    print(f"Batch  : {cfg['batch']}")
    print()

    result = KSBSimulation(cfg=cfg).run(seed=args.seed)

    # ── Slot assignment ──────────────────────────────────────────────────────
    print("=== Slot assignment ===")
    skips = result.skip_indices + 1
    if len(skips) == 0:
        print("  Skipped slots  : none")
    else:
        print(f"  Skipped slots  : {len(skips)} skip(s) before item(s) {skips + 1}")

    # ── Time horizons ────────────────────────────────────────────────────────
    print()
    print("=== Time horizons (buffer correction window) ===")
    th = result.time_horizons
    print(f"  min  : {th.min():.3f} s")
    print(f"  mean : {th.mean():.3f} s")
    print(f"  max  : {th.max():.3f} s")

    # ── Phase errors ─────────────────────────────────────────────────────────
    print()
    print("=== Phase errors ===")
    print(f"  φ_u  (pre-buffer, buckets) : mean={result.phi_u.mean():.3f}  "
          f"std={result.phi_u.std():.3f}  "
          f"range=[{result.phi_u.min():.3f}, {result.phi_u.max():.3f}]")
    print(f"  φ_0  (at entry,  buckets)  : mean={result.phi_0.mean():.3f}  "
          f"std={result.phi_0.std():.3f}  "
          f"range=[{result.phi_0.min():.3f}, {result.phi_0.max():.3f}]")

    # ── Gap metrics ───────────────────────────────────────────────────────────
    if (len(result.pair_records) > 0):
        print()
        print("=== Gap metrics (consecutive pairs on buffer) ===")
        pairs = result.pair_records
        min_gaps = np.array([p.min_gap for p in pairs])
        avg_margins = np.array([p.average_margin for p in pairs
                                if p.average_margin is not None])
        viol_integrals = np.array([p.violation_integral for p in pairs
                                if p.violation_integral is not None])

        p_min_threshold = pairs[0].g_min_threshold if pairs else float("nan")
        print(f"  p_min threshold            : {p_min_threshold:.3f} m")
        print(f"  Min instantaneous gap    : {min_gaps.min():.4f} m  "
            f"(pair {int(np.argmin(min_gaps))})")
        print(f"  Mean min gap per pair    : {min_gaps.mean():.4f} m")
        n_violating = int(np.sum(min_gaps < p_min_threshold))
        print(f"  Pairs violating p_min      : {n_violating} / {len(pairs)}")
        if len(viol_integrals) > 0:
            print(f"  Total violation integral   : {viol_integrals.sum():.6f} m·s")
        if len(avg_margins) > 0:
            print(f"  Mean average margin        : {avg_margins.mean():.4f} m")


if __name__ == "__main__":
    main()
