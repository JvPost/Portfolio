from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from ruckig import ControlInterface, InputParameter, Result, Ruckig, Trajectory

from ksb.analysis.events import SegmentEvents


@dataclass
class SegmentSyncResponse:
    """Per-(pair, segment) minimum-time synchronization response.

    Wraps Ruckig (velocity interface) to compute T_min for each ξ_{i,k},
    and derives the kinematic margin against the free window W from
    SegmentEvents.

    Args:
        events: SegmentEvents with ξ = (v⁻, a⁻, v⁺, a⁺) and W per (i, k).
        bounds: np.ndarray shape (3,), ordered [v_max, a_max, j_max].

    Attributes (all shape (b-1, N_B), matching events.W):
        T_min: Ruckig-computed minimum trajectory duration.
        kinematic_margin: events.W - T_min.
        feasible: kinematic_margin >= 0.
    """

    T_min: np.ndarray
    kinematic_margin: np.ndarray
    state_error: np.ndarray
    feasible: np.ndarray

    def __init__(self, events: SegmentEvents, bounds: np.ndarray) -> None:
        assert bounds.shape == (2,), f"bounds must have shape (2,), got {bounds.shape}"
        a_max, j_max = bounds
        n_pairs, N_B = events.W.shape

        T_min = np.empty_like(events.W)
        # last axis is (Δv, Δa); init NaN = undefined (co-occupancy cells, set below)
        state_error = np.full((n_pairs, N_B, 2), np.nan)

        otg = Ruckig(1)
        inp = InputParameter(1)
        inp.control_interface = ControlInterface.Velocity
        inp.current_position = [0.0]
        inp.max_acceleration = [float(a_max)]
        inp.max_jerk = [float(j_max)]
        traj = Trajectory(1)

        
        for i in range(n_pairs):
            for k in range(N_B):
                inp.current_velocity = [float(events.v_out[i, k])]
                inp.current_acceleration = [float(events.a_out[i, k])]
                inp.target_velocity = [float(events.v_in[i + 1, k])]
                inp.target_acceleration = [float(events.a_in[i + 1, k])]

                result = otg.calculate(inp, traj)
                if result != Result.Working:
                    raise RuntimeError(
                        f"Ruckig failed at (i={i}, k={k}): "
                        f"ξ = (v_out={events.v_out[i, k]:.6g}, a_out={events.a_out[i, k]:.6g}, "
                        f"v_in={events.v_in[i + 1, k]:.6g}, a_in={events.a_in[i + 1, k]:.6g}), "
                        f"bounds = [a_max={a_max:.6g}, j_max={j_max:.6g}], "
                        f"result={result}"
                    )

                w = events.W[i,k]
                if (w < traj.duration and w >= 0):
                    x_in = events.x_in[i+1, k] # state of incoming input
                    x_S = np.array(traj.at_time(w)).squeeze() # state of \pi^S
                    s_error = x_in[1:] - x_S[1:] # only vel and acc matter
                    state_error[i, k] = s_error

                    
                    # State error is a sufficient statistic for slip: the map
                    # (Δv, Δa) -> slip is monotone and nonlinear, so argmin of state
                    # error equals argmin of slip. We never need the slip function
                    # itself for optimization, only the state error computed here.

                T_min[i, k] = traj.duration


        self.T_min = T_min
        self.kinematic_margin = events.W - T_min
        self.feasible = self.kinematic_margin >= 0

        # Three-band partition of state_error's last axis (Δv, Δa):
        #   W < 0            co-occupancy   -> NaN  (no transition; undefined)
        #   0 <= W < T_min   sync shortfall -> finite, computed in the loop above
        #   W >= T_min       feasible       -> 0    (target reached within window)
        self.concurrency_error = events.W < 0
        state_error[self.feasible] = 0.0
        self.state_error = state_error

        # Partition invariant: the undefined (NaN) cells are exactly the
        # co-occupancy cells — nothing else should be left unwritten.
        assert np.array_equal(
            np.isnan(state_error[..., 0]), self.concurrency_error
        )
        assert np.array_equal(
            np.isnan(state_error[..., 1]), self.concurrency_error
        )
        assert np.all(
            state_error[~self.feasible, 0] != 0
        )
        assert np.all(
            state_error[~self.feasible, 1] != 0
        )
