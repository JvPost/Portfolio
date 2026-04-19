import numpy as np
import pytest

from ksb.analysis.events import compute_segment_events
from ksb.simulation.ksb_simulation import KSBSimulation


class TestSegmentEvents:
    """Tests for SegmentEvents computation."""

    def get_default_cfg(self, n_buffer_seg=5):
        """Return a standard config for testing."""
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
            "slot_length": 0.6,
            "input_gap_mean": 0.6,
            "input_gap_std": 0.05,
            "arrival_rate_ppm": 120,
            "slot_rate_ppm": 132,
            "batch": 10,
            "solver": "quintic",
            "v_buff_out": 1.5,
        }

    def test_monotonicity_t_out(self):
        """Test (A): t_out strictly increasing per pair (segments ordered)."""
        cfg = self.get_default_cfg(n_buffer_seg=5)
        sim = KSBSimulation(cfg=cfg)
        result = sim.run(seed=42)

        assert result.segment_events is not None, "segment_events is None for batch > 1"

        t_out = result.segment_events.t_out  # shape (b-1, N_B)
        # For each pair i, t_out[i, :] should be strictly increasing
        for i in range(t_out.shape[0]):
            diffs = np.diff(t_out[i, :])
            assert np.all(diffs > 1e-9), (
                f"t_out[{i}, :] is not strictly increasing: {t_out[i, :]}"
            )

    def test_monotonicity_t_in(self):
        """Test (A): t_in strictly increasing per pair (segments ordered)."""
        cfg = self.get_default_cfg(n_buffer_seg=5)
        sim = KSBSimulation(cfg=cfg)
        result = sim.run(seed=42)

        assert result.segment_events is not None, "segment_events is None for batch > 1"

        t_in = result.segment_events.t_in  # shape (b-1, N_B)
        # For each pair i, t_in[i, :] should be strictly increasing
        for i in range(t_in.shape[0]):
            diffs = np.diff(t_in[i, :])
            assert np.all(diffs > 1e-9), (
                f"t_in[{i}, :] is not strictly increasing: {t_in[i, :]}"
            )

    def test_W_finite(self):
        """Test that budget matrix W contains no NaN or inf values."""
        cfg = self.get_default_cfg(n_buffer_seg=5)
        sim = KSBSimulation(cfg=cfg)
        result = sim.run(seed=42)

        assert result.segment_events is not None
        W = result.segment_events.W
        assert np.all(np.isfinite(W)), (
            f"Budget matrix W should be finite. Found {np.sum(~np.isfinite(W))} non-finite values"
        )

    def test_boundary_invariance_n_b_single_vs_four(self):
        """Test (B): Boundary events are invariant to N_B (N_B=1 vs N_B=4)."""
        seed = 42
        batch = 5

        # Run with N_B = 1
        cfg_1 = self.get_default_cfg(n_buffer_seg=1)
        cfg_1["batch"] = batch
        sim_1 = KSBSimulation(cfg=cfg_1)
        result_1 = sim_1.run(seed=seed)

        # Run with N_B = 4
        cfg_4 = self.get_default_cfg(n_buffer_seg=4)
        cfg_4["batch"] = batch
        sim_4 = KSBSimulation(cfg=cfg_4)
        result_4 = sim_4.run(seed=seed)

        assert result_1.segment_events is not None
        assert result_4.segment_events is not None

        # Boundary events should match:
        # t_in[:, 0] at N_B=1 == t_in[:, 0] at N_B=4 (follower buffer entry)
        np.testing.assert_allclose(
            result_1.segment_events.t_in[:, 0],
            result_4.segment_events.t_in[:, 0],
            rtol=1e-9,
            atol=1e-12,
            err_msg="Follower buffer-entry event should not depend on N_B",
        )

        # t_out[:, N_B-1] at N_B=1 == t_out[:, 3] at N_B=4 (leader buffer exit)
        np.testing.assert_allclose(
            result_1.segment_events.t_out[:, 0],
            result_4.segment_events.t_out[:, 3],
            rtol=1e-9,
            atol=1e-12,
            err_msg="Leader buffer-exit event should not depend on N_B",
        )

    def test_segment_events_none_for_batch_1(self):
        """Test that segment_events is None for batch = 1."""
        cfg = self.get_default_cfg()
        cfg["batch"] = 1
        sim = KSBSimulation(cfg=cfg)
        result = sim.run(seed=42)

        assert result.segment_events is None, "segment_events should be None for batch < 2"

    def test_shape_consistency(self):
        """Test that segment_events arrays have correct shape (b-1, N_B)."""
        cfg = self.get_default_cfg(n_buffer_seg=5)
        batch = 8
        cfg["batch"] = batch
        sim = KSBSimulation(cfg=cfg)
        result = sim.run(seed=42)

        assert result.segment_events is not None
        events = result.segment_events

        expected_shape = (batch - 1, sim.n_buffer_seg)
        assert events.t_out.shape == expected_shape
        assert events.t_in.shape == expected_shape
        assert events.v_minus.shape == expected_shape
        assert events.a_minus.shape == expected_shape
        assert events.v_plus.shape == expected_shape
        assert events.a_plus.shape == expected_shape
        assert events.W.shape == expected_shape
