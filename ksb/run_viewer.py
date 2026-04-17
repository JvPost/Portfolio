#!/usr/bin/env python3
"""Launch the KSB pygame viewer.

Usage:
    python run_viewer.py [--seed INT] [--config NAME]
"""
import argparse
from pathlib import Path

import yaml

from ksb.planning.solvers.quintic import QuinticSolver
from ksb.planning.solvers.scurve import SCurveSolver
from ksb.simulation.ksb_simulation import KSBSimulation
from ksb.viewer.viewer import KSBViewer

_CONFIG_DIR = Path(__file__).parent / "configs"


def main():
    parser = argparse.ArgumentParser(description="KSB pygame viewer")
    parser.add_argument("--seed", type=int, default=42,
                        help="Random seed (default: 42)")
    parser.add_argument("--config", type=str, default="default",
                        help="Config file in configs/ without .yaml suffix (default: default)")
    args = parser.parse_args()

    with open(_CONFIG_DIR / f"{args.config}.yaml") as f:
        cfg = yaml.safe_load(f)

    solver_name = cfg.get("solver", "scurve")

    print(f"Running simulation: solver={solver_name}  seed={args.seed}  "
          f"batch={cfg['batch']}  std={cfg['input_gap_std']}")
    result = KSBSimulation(cfg=cfg).run(seed=args.seed)
    print(f"  assigned_slots : {result.assigned_slots}")
    print(f"  skip_indices   : {result.skip_indices}")
    print(f"  violations     : {sum(1 for p in result.pair_records if p.min_gap is not None and p.min_gap < cfg['L_buffer'] / cfg['n_buffer_seg'] * 2)}")
    print("Launching viewer…  SPACE to start, ESC to quit.")

    KSBViewer(result, cfg).run()


if __name__ == "__main__":
    main()
