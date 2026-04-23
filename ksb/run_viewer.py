#!/usr/bin/env python3
"""Launch the KSB pygame viewer.

Usage:
    python run_viewer.py [--seed INT] [--config NAME] [--with-analysis]
"""
import argparse
from pathlib import Path

import yaml

from ksb.planning.solvers.quintic import QuinticSolver
from ksb.planning.solvers.scurve import SCurveSolver
from ksb.simulation.ksb_simulation import KSBSimulation
from ksb.viewer.viewer import KSBViewer
from ksb.analysis.events import compute_segment_events
from ksb.analysis.cost import compute_C_bb

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

    events = None
    cost = None
    events = compute_segment_events(
        total_trajectories=result.composite_trajectories,
        t_spawn=result.t_spawn,
        input_length=float(cfg.get("input_length", 0.32)),
        L_upstream=float(cfg.get("L_upstream", 1.0)),
        L_buffer=float(cfg.get("L_buffer", 2.0)),
        N_B=int(cfg.get("n_buffer_seg", 5)),
    )
    cost = compute_C_bb(events, float(cfg.get("jmax", 50.0)))

    print("Launching viewer…  SPACE to start, ESC to quit.")

    KSBViewer(result, cfg, events=events, cost=cost).run()


if __name__ == "__main__":
    main()
