#!/usr/bin/env python3
"""Launch the KSB viewer using the best optimizer result from a run directory.

Usage:
    python run_viewer_best.py --run-dir results/<tag>  [--seed INT] [--speed FLOAT]
"""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml

from ksb.simulation.ksb_simulation import KSBSimulation
from ksb.viewer.viewer import KSBViewer
from ksb.analysis.cost import compute_C_bb, compute_S_bb


def _load_best(run_dir: Path) -> tuple[dict, dict]:
    """Return (merged_cfg, theta_star) for the NB with the lowest L_star."""
    with open(run_dir / "run_info.yaml") as f:
        info = yaml.safe_load(f)
    system_cfg = dict(info["system_cfg"])

    inner_files = sorted(run_dir.glob("inner_NB_*.json"))
    if not inner_files:
        raise FileNotFoundError(f"No inner_NB_*.json files found in {run_dir}")

    best_inner = min(
        (json.load(open(p)) for p in inner_files),
        key=lambda d: d["L_star"],
    )

    theta = best_inner["theta_star"]
    cfg = {**system_cfg}
    cfg["n_buffer_seg"] = best_inner["n_buffer_seg"]
    cfg.update({k: theta[k] for k in ("L_buffer", "beta", "gamma", "eta_r", "eta_s", "eta_v") if k in theta})
    return cfg, theta


def main() -> None:
    parser = argparse.ArgumentParser(description="KSB viewer — best optimizer result")
    parser.add_argument("--run-dir", required=True, help="Path to optimizer run directory")
    parser.add_argument("--seed",    type=int,   default=42)
    parser.add_argument("--speed",   type=float, default=1.0)
    args = parser.parse_args()

    run_dir = Path(args.run_dir)
    cfg, theta = _load_best(run_dir)

    nb  = cfg["n_buffer_seg"]
    bet = theta.get("beta",  0.0)
    gam = theta.get("gamma", 0.0)
    print(f"Run dir : {run_dir}")
    print(f"NB={nb}  beta={bet:.4f}  gamma={gam:.4f}  L_buffer={cfg['L_buffer']:.4f}")
    print(f"solver={cfg.get('solver','?')}  seed={args.seed}  batch={cfg['batch']}")

    result = KSBSimulation(cfg=cfg).run(seed=args.seed)
    print(f"  assigned_slots : {result.assigned_slots}")
    print(f"  skip_indices   : {result.skip_indices}")

    events = result.segment_events
    cost   = compute_C_bb(events, float(cfg.get("jmax", 50.0)))
    slack  = compute_S_bb(events, float(cfg.get("jmax", 50.0)))
    print(f"  violations     : {np.sum(slack < 0)}")
    print("Launching viewer…  SPACE to start, ESC to quit.")

    KSBViewer(result, cfg, speed=args.speed, events=events, cost=cost).run()


if __name__ == "__main__":
    main()
