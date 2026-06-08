#!/usr/bin/env python3
"""Run a KSB simulation and print a summary to stdout.

Usage:
    python run_sim.py [--seed INT] [--config NAME]
"""
import argparse
import time
from pathlib import Path

import numpy as np
import yaml

from ksb.planning.solvers.quintic import QuinticSolver
from ksb.planning.solvers.scurve import SCurveSolver
from ksb.simulation.ksb_simulation import KSBSimulation

_CONFIG_DIR = Path(__file__).parent / "configs" / "system"


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

    start_time = time.time()
    result = KSBSimulation(cfg=cfg).run(seed=args.seed)
    elapsed = time.time() - start_time
    print(f"Simulation time : {elapsed:.3f} s")
    print()

    # ── Slot assignment ──────────────────────────────────────────────────────
    print("=== Slot assignment ===")
    skips = result.skip_indices + 1
    if len(skips) == 0:
        print("  Skipped slots  : none")
    else:
        print(f"  Skipped slots  : {len(skips)} skip(s) after item(s) {skips}")

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
    print(f"  φ_b  (pre-buffer, buckets) : mean={result.phi_b.mean():.3f}  "
          f"std={result.phi_b.std():.3f}  "
          f"range=[{result.phi_b.min():.3f}, {result.phi_b.max():.3f}]")

    if (result.segment_sync_response):
        print()
        print("=== Segment boundary events (i, k) ===")
         
        n_violating = result.segment_events.W < 0
        n_feasible = result.segment_sync_response.feasible
        size = n_feasible.size
        print(f"fraction violations: {np.sum(n_violating) / size}")
        print(f"fraction feasible events: {np.sum(n_feasible) / size}")

if __name__ == "__main__":
    main()
