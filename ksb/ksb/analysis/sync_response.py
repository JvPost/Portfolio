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
    feasible: np.ndarray

    def __init__(self, events: SegmentEvents, bounds: np.ndarray) -> None:
        assert bounds.shape == (2,), f"bounds must have shape (2,), got {bounds.shape}"
        a_max, j_max = bounds

        T_min = np.empty_like(events.W)

        otg = Ruckig(1)
        inp = InputParameter(1)
        inp.control_interface = ControlInterface.Velocity
        inp.current_position = [0.0]
        inp.max_acceleration = [float(a_max)]
        inp.max_jerk = [float(j_max)]
        traj = Trajectory(1)

        n_pairs, N_B = events.W.shape
        for i in range(n_pairs):
            for k in range(N_B):
                inp.current_velocity = [float(events.v_minus[i, k])]
                inp.current_acceleration = [float(events.a_minus[i, k])]
                inp.target_velocity = [float(events.v_plus[i, k])]
                inp.target_acceleration = [float(events.a_plus[i, k])]

                result = otg.calculate(inp, traj)
                if result != Result.Working:
                    raise RuntimeError(
                        f"Ruckig failed at (i={i}, k={k}): "
                        f"ξ = (v⁻={events.v_minus[i, k]:.6g}, a⁻={events.a_minus[i, k]:.6g}, "
                        f"v⁺={events.v_plus[i, k]:.6g}, a⁺={events.a_plus[i, k]:.6g}), "
                        f"bounds = [a_max={a_max:.6g}, j_max={j_max:.6g}], "
                        f"result={result}"
                    )
                T_min[i, k] = traj.duration

        self.T_min = T_min
        self.kinematic_margin = events.W - T_min
        self.feasible = self.kinematic_margin >= 0
