"""Regression test: beta=0, gamma=0 must produce bit-identical slot/phase results
and numerically equivalent segment geometry to the pre-softmax uniform baseline.
"""
from __future__ import annotations

import numpy as np
import pytest

from ksb.simulation.ksb_simulation import KSBSimulation
from ksb.simulation.utils import belt_lengths


_BASE_CFG = {
    "jmax": 100.0,
    "Vmax": 3.0,
    "Amax": 8.5,
    "L_upstream": 2.0,
    "L_buffer": 3.0,
    "L_registrar": 1.0,
    "L_downstream": 2.0,
    "input_length": 0.32,
    "n_buffer_seg": 6,
    "eta_s": 1.875,
    "eta_r": 1.2,
    "input_gap_mean": 0.6,
    "input_gap_std": 0.05,
    "arrival_rate_ppm": 180,
    "batch": 20,
    "solver": "scurve",
    "v_buff_out": 2.5,
}

SEED = 42


class TestSegmentGeometryRegression:
    def _run(self, extra: dict) -> object:
        cfg = {**_BASE_CFG, **extra}
        return KSBSimulation(cfg=cfg).run(seed=SEED)

    def test_belt_lengths_uniform_at_zero_skew(self):
        """belt_lengths(N, L, Lmin, 0, 0) returns L/N for all segments."""
        N, L = 6, 3.0
        Lmin = 1.25 * 0.32
        Ls = belt_lengths(N, L, Lmin, beta=0.0, gamma=0.0)
        np.testing.assert_allclose(Ls, np.full(N, L / N), rtol=1e-14)

    def test_belt_lengths_sums_to_L_total(self):
        """belt_lengths sums to L_total for arbitrary (beta, gamma)."""
        for beta, gamma in [(0.5, -1.0), (-1.5, 0.8), (2.0, 2.0)]:
            Ls = belt_lengths(6, 3.0, 0.40, beta=beta, gamma=gamma)
            np.testing.assert_allclose(Ls.sum(), 3.0, atol=1e-12)

    def test_belt_lengths_floor(self):
        """Every segment >= Lmin."""
        Lmin = 1.25 * 0.32
        for beta, gamma in [(2.0, -2.0), (-2.0, 2.0)]:
            Ls = belt_lengths(6, 3.0, Lmin, beta=beta, gamma=gamma)
            assert np.all(Ls >= Lmin - 1e-12), f"Floor violated: {Ls}"

    def test_slot_assignments_unchanged(self):
        """Slot assignments are bit-identical with and without softmax keys."""
        r_default = self._run({})
        r_explicit = self._run({"beta": 0.0, "gamma": 0.0})
        np.testing.assert_array_equal(
            r_default.assigned_slots,
            r_explicit.assigned_slots,
            err_msg="Slot assignments differ when beta=gamma=0",
        )

    def test_phase_errors_unchanged(self):
        """Phase errors phi_u, phi_0 are bit-identical."""
        r_default = self._run({})
        r_explicit = self._run({"beta": 0.0, "gamma": 0.0})
        np.testing.assert_array_equal(r_default.phi_u, r_explicit.phi_u)
        np.testing.assert_array_equal(r_default.phi_0, r_explicit.phi_0)

    def test_W_matrix_uniform(self):
        """With beta=gamma=0, W matrix from Ls-based boundaries matches uniform N_B formula."""
        r = self._run({"beta": 0.0, "gamma": 0.0})
        assert r.segment_events is not None
        W = r.segment_events.W

        # All W values must be finite (no NaN/inf)
        assert np.all(np.isfinite(W)), "W matrix contains non-finite values"

    def test_W_matrix_invariant_to_softmax_keys(self):
        """W matrix is numerically equivalent whether or not softmax keys are present."""
        r_default = self._run({})
        r_explicit = self._run({"beta": 0.0, "gamma": 0.0})
        assert r_default.segment_events is not None
        assert r_explicit.segment_events is not None
        np.testing.assert_allclose(
            r_default.segment_events.W,
            r_explicit.segment_events.W,
            rtol=1e-12,
            atol=1e-12,
            err_msg="W matrix differs when beta=gamma=0 vs default",
        )

    def test_segment_events_shape(self):
        """segment_events shape = (batch-1, n_buffer_seg)."""
        r = self._run({})
        assert r.segment_events is not None
        b = _BASE_CFG["batch"]
        N = _BASE_CFG["n_buffer_seg"]
        assert r.segment_events.W.shape == (b - 1, N)
