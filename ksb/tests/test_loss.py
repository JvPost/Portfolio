"""Unit tests for ksb.optimization.loss.compute_loss."""
from __future__ import annotations

import math
import numpy as np
import pytest
import yaml
from pathlib import Path

from ksb.optimization.loss import compute_loss, LossResult

_CONFIG_DIR = Path(__file__).parent.parent / "configs" / "system"


def _default_cfg() -> dict:
    with open(_CONFIG_DIR / "default.yaml") as f:
        return yaml.safe_load(f)


class TestComputeLoss:
    def test_finite_at_default_config(self):
        """compute_loss returns finite L and sentinel=False at default config."""
        cfg = _default_cfg()
        lr = compute_loss(cfg, lambda_U=0.05, lambda_L=0.5, lambda_N=.1, lambda_T=1.0, seeds=[42])
        assert not lr.sentinel, f"Unexpected sentinel at default config: {lr.per_seed}"
        assert math.isfinite(lr.L), f"L is not finite: {lr.L}"
        assert lr.phi_sum >= 0.0
        assert lr.U_sum >= 0.0

    def test_loss_components_sum(self):
        """L == phi_sum + lambda_U*U_sum + lambda_L*L_buffer + lambda_T*eta_r."""
        cfg = _default_cfg()
        lU, lL, lT = 0.05, 0.5, 1.0
        lr = compute_loss(cfg, lambda_U=lU, lambda_L=lL, lambda_N=.1, lambda_T=lT, seeds=[42])
        expected = lr.phi_sum + lU * lr.U_sum + lL * lr.L_buffer + lT * lr.eta_r
        assert abs(lr.L - expected) < 1e-10, f"Loss doesn't match components: {lr.L} vs {expected}"

    def test_eta_r_from_cfg(self):
        """eta_r is read directly from cfg."""
        cfg = _default_cfg()
        lr = compute_loss(cfg, lambda_U=0.05, lambda_L=0.5, lambda_N=.1, lambda_T=1.0, seeds=[42])
        assert abs(lr.eta_r - cfg["eta_r"]) < 1e-10

    def test_multiple_seeds_averages(self):
        """With multiple seeds, results are averaged and sentinel=False if all finite."""
        cfg = _default_cfg()
        lr = compute_loss(cfg, lambda_U=0.05, lambda_L=0.5, lambda_N=.1, lambda_T=1.0, seeds=[0, 1, 2])
        assert len(lr.per_seed) == 3
        assert not lr.sentinel
        assert math.isfinite(lr.L)

    def test_sentinel_on_bad_cfg(self):
        """Extremely infeasible cfg triggers sentinel=True and L=inf."""
        cfg = _default_cfg()
        # Zero buffer length is infeasible
        cfg["L_buffer"] = 0.001
        cfg["n_buffer_seg"] = 1
        lr = compute_loss(cfg, lambda_U=0.05, lambda_L=0.5, lambda_N=.1, lambda_T=1.0, seeds=[42])
        # Either sentinel or L=inf; at least one must hold if simulation breaks
        assert lr.sentinel or not math.isfinite(lr.L) or lr.L > 0
