"""Tests for RuckigSolver and RuckigTrajectory."""
from __future__ import annotations

import numpy as np
import pytest

from ksb.motion.trajectories import P, V, A
from ksb.planning.contracts import InfeasibleError, J_MAX, A_MAX, V_MAX, Policy
from ksb.planning.solvers.ruckig_solver import RuckigSolver, RuckigTrajectory
from ksb.planning.solvers.scurve import SCurveSolver

# Shared constants — j_max, a_max, v_max, pitch_min (PITCH_MIN unused by Ruckig)
BOUNDS = np.array([50.0, 8.5, 3.0, 0.0])
POLICY = Policy()
SOLVER = RuckigSolver()

# Representative feasible BCs: forward motion, no direction reversal
PI, VI, PF, VF = 0.0, 2.0, 1.0, 2.5
T_FEASIBLE = 1.0  # well above T_min (~0.40 s for these BCs)


class TestRuckigSolverRoundTrip:
    def _solve(self, T: float = T_FEASIBLE) -> RuckigTrajectory:
        return SOLVER.solve(PI, VI, PF, VF, T, BOUNDS, POLICY)

    def test_eval_at_zero_initial_state(self):
        traj = self._solve()
        s = traj.eval(0.0)
        assert abs(s[P]) < 1e-6
        assert abs(s[V] - VI) < 1e-6
        assert abs(s[A]) < 1e-6

    def test_eval_at_T_target_state(self):
        traj = self._solve()
        s = traj.eval(traj.T)
        assert abs(s[P] - (PF - PI)) < 1e-6
        assert abs(s[V] - VF) < 1e-6
        assert abs(s[A]) < 1e-5

    def test_duration_equals_T(self):
        traj = self._solve()
        assert abs(traj.T - T_FEASIBLE) < 1e-9

    def test_eval_scalar_shape(self):
        traj = self._solve()
        assert traj.eval(0.0).shape == (3,)

    def test_eval_array_shape(self):
        traj = self._solve()
        out = traj.eval(np.linspace(0, traj.T, 20))
        assert out.shape == (3, 20)

    def test_get_duration(self):
        traj = self._solve()
        assert traj.get_duration() == traj.T

    def test_check_bounds_always_true(self):
        traj = self._solve()
        assert traj.check_bounds(BOUNDS) is True


class TestRuckigSolverInfeasibility:
    def test_T_far_below_T_min(self):
        with pytest.raises(InfeasibleError):
            SOLVER.solve(PI, VI, PF, VF, T=1e-6, bounds=BOUNDS, policy=POLICY)

    def test_large_displacement_tiny_T(self):
        # 10 m at v_max=3 needs at least 3.3 s; T=0.01 is clearly infeasible
        with pytest.raises(InfeasibleError):
            SOLVER.solve(0.0, 2.0, 10.0, 2.0, T=0.01, bounds=BOUNDS, policy=POLICY)


class TestFindTimeAtPosition:
    def _traj(self) -> RuckigTrajectory:
        return SOLVER.solve(PI, VI, PF, VF, T_FEASIBLE, BOUNDS, POLICY)

    def test_p_zero_returns_zero(self):
        t = self._traj().find_time_at_position(0.0)
        assert t is not None
        assert abs(t) < 1e-9

    def test_p_end_returns_T(self):
        traj = self._traj()
        pT = traj.eval(traj.T)[P]
        t = traj.find_time_at_position(pT)
        assert t is not None
        assert abs(t - traj.T) < 1e-9

    def test_above_range_returns_none(self):
        traj = self._traj()
        pT = traj.eval(traj.T)[P]
        assert traj.find_time_at_position(pT + 1.0) is None

    def test_below_range_returns_none(self):
        assert self._traj().find_time_at_position(-1.0) is None

    def test_mid_trajectory_round_trip(self):
        traj = self._traj()
        t_star = traj.T * 0.4
        p_star = traj.eval(t_star)[P]
        t_found = traj.find_time_at_position(p_star)
        assert t_found is not None
        assert abs(t_found - t_star) < 1e-6


class TestRuckigVsSCurve:
    """RuckigSolver and SCurveSolver must reach the same endpoint at matched T."""

    def test_target_state_agreement(self):
        scurve = SCurveSolver()

        T_min, _ = scurve.feasibility_window(PI, VI, PF, VF, BOUNDS, POLICY)
        T = T_min + 0.2  # comfortably feasible for both solvers

        traj_sc = scurve.solve(PI, VI, PF, VF, T, BOUNDS, POLICY)
        traj_rk = SOLVER.solve(PI, VI, PF, VF, T, BOUNDS, POLICY)

        end_sc = traj_sc.eval(traj_sc.T)
        end_rk = traj_rk.eval(traj_rk.T)

        np.testing.assert_allclose(end_sc[P], end_rk[P], atol=1e-6)
        np.testing.assert_allclose(end_sc[V], end_rk[V], atol=1e-6)
        np.testing.assert_allclose(end_sc[A], end_rk[A], atol=1e-5)
