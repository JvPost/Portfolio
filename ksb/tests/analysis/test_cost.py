import numpy as np
import pytest

from ksb.analysis.cost import (
    BBCostResult,
    compute_C_bb,
    compute_Phi_bb,
    compute_S_bb,
)
from ksb.analysis.events import SegmentEvents
from ksb.simulation.ksb_simulation import KSBSimulation


def _make_events(v_minus, a_minus, v_plus, a_plus, t_out=None, t_in=None):
    """Build a SegmentEvents from 1x1 synthetic BC values (or larger arrays)."""
    v_minus = np.atleast_2d(np.asarray(v_minus, dtype=float))
    a_minus = np.atleast_2d(np.asarray(a_minus, dtype=float))
    v_plus = np.atleast_2d(np.asarray(v_plus, dtype=float))
    a_plus = np.atleast_2d(np.asarray(a_plus, dtype=float))
    if t_out is None:
        t_out = np.zeros_like(v_minus)
    if t_in is None:
        t_in = np.zeros_like(v_minus)
    return SegmentEvents(
        t_out=t_out,
        t_in=t_in,
        v_minus=v_minus,
        a_minus=a_minus,
        v_plus=v_plus,
        a_plus=a_plus,
    )


def _default_cfg(n_buffer_seg=5):
    return {
        "jmax": 100.0,
        "Vmax": 3.0,
        "Amax": 8.5,
        "L_upstream": 2.0,
        "L_buffer": 3.0,
        "L_registrar": 0.7,
        "L_downstream": 2.0,
        "input_length": 0.32,
        "n_buffer_seg": n_buffer_seg,
        "eta_s": 1.875,
        "eta_r": 1.1,
        "input_gap_mean": 0.6,
        "input_gap_std": 0.05,
        "arrival_rate_ppm": 120,
        "batch": 10,
        "solver": "quintic",
        "v_buff_out": 1.5,
    }


class TestBBCost:
    """Tests for bang-bang minimum transition time cost."""

    def test_zero_bc_positive_dv(self):
        """Zero-BC reduction, positive dv: C = 2*sqrt(dv/j_max), up-case wins."""
        j_max = 4.0
        dv = 1.0
        events = _make_events(v_minus=0.0, a_minus=0.0, v_plus=dv, a_plus=0.0)
        res = compute_C_bb(events, j_max)

        expected = 2.0 * np.sqrt(dv / j_max)
        np.testing.assert_allclose(res.C, expected, rtol=1e-12, atol=0)
        assert res.case.item() == 1
        assert res.feasible.item()

    def test_zero_bc_negative_dv(self):
        """Zero-BC reduction, negative dv: C = 2*sqrt(|dv|/j_max), down-case wins."""
        j_max = 4.0
        dv = -1.0
        events = _make_events(v_minus=0.0, a_minus=0.0, v_plus=dv, a_plus=0.0)
        res = compute_C_bb(events, j_max)

        expected = 2.0 * np.sqrt(abs(dv) / j_max)
        np.testing.assert_allclose(res.C, expected, rtol=1e-12, atol=0)
        assert res.case.item() == -1
        assert res.feasible.item()

    def test_zero_bc_zero_dv(self):
        """Zero-BC, dv = 0: C = 0, a_peak = 0."""
        events = _make_events(v_minus=0.0, a_minus=0.0, v_plus=0.0, a_plus=0.0)
        res = compute_C_bb(events, j_max=1.0)

        np.testing.assert_allclose(res.C, 0.0, atol=1e-15)
        np.testing.assert_allclose(res.a_peak, 0.0, atol=1e-15)
        assert res.feasible.item()

    def test_symmetric_bc_zero_dv(self):
        """Symmetric BCs a_minus = a_plus = 5, dv = 0: up-case at boundary wins with C=0."""
        # Under weak feasibility (>=), the up case is feasible at a_p_up = max(a_m, a_p) = 5,
        # giving tau1 = tau2 = 0 and C = 0 (BCs already match, no transition needed).
        # The down case is also feasible with C = 20/j_max, but the minimum is 0.
        j_max = 1.0
        events = _make_events(v_minus=0.0, a_minus=5.0, v_plus=0.0, a_plus=5.0)
        res = compute_C_bb(events, j_max)

        np.testing.assert_allclose(res.C, 0.0, atol=1e-15)
        assert res.case.item() == 1
        assert res.feasible.item()

    def test_antisymmetric_bc(self):
        """Anti-symmetric BCs: both cases feasible, both give C = 10/j_max."""
        j_max = 1.0
        events = _make_events(v_minus=0.0, a_minus=5.0, v_plus=0.0, a_plus=-5.0)
        res = compute_C_bb(events, j_max)

        np.testing.assert_allclose(res.C, 10.0 / j_max, rtol=1e-12)
        assert res.feasible.item()
        # Both cases feasible and agree; either case selection is valid
        assert res.case.item() in (1, -1)

    def test_full_sim_smoke(self):
        """End-to-end: compute_C_bb on a simulated SegmentEvents yields finite, non-negative C."""
        cfg = _default_cfg(n_buffer_seg=5)
        sim = KSBSimulation(cfg=cfg)
        result = sim.run(seed=42)

        assert result.segment_events is not None
        res = compute_C_bb(result.segment_events, cfg["jmax"])

        assert res.C.shape == result.segment_events.W.shape
        assert np.all(np.isfinite(res.C)), (
            f"C has non-finite values: {np.sum(~np.isfinite(res.C))} cells infeasible"
        )
        assert res.C.min() >= 0.0
        assert np.all(res.feasible)

    def test_shape_consistency(self):
        """All BBCostResult arrays share shape (b-1, N_B), matching W."""
        cfg = _default_cfg(n_buffer_seg=5)
        batch = 8
        cfg["batch"] = batch
        sim = KSBSimulation(cfg=cfg)
        result = sim.run(seed=42)

        assert result.segment_events is not None
        events = result.segment_events
        expected_shape = (batch - 1, sim.n_buffer_seg)

        res = compute_C_bb(events, cfg["jmax"])
        assert res.C.shape == events.W.shape == expected_shape
        assert res.a_peak.shape == expected_shape
        assert res.case.shape == expected_shape
        assert res.feasible.shape == expected_shape

        S = compute_S_bb(events, cfg["jmax"])
        Phi = compute_Phi_bb(events, cfg["jmax"])
        assert S.shape == expected_shape
        assert Phi.shape == expected_shape

    def test_slack_diagnostic(self):
        """Diagnostic: fraction of negative-slack cells (edge-concentrated per formalization §3.4)."""
        cfg = _default_cfg(n_buffer_seg=5)
        sim = KSBSimulation(cfg=cfg)
        result = sim.run(seed=42)

        assert result.segment_events is not None
        S = compute_S_bb(result.segment_events, cfg["jmax"])
        neg_frac = float((S < 0).mean())
        print(f"\n[diagnostic] fraction of S < 0 cells: {neg_frac:.4f}")
        print(f"[diagnostic] min S by segment index: {S.min(axis=0)}")
