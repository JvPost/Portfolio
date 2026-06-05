from __future__ import annotations

from dataclasses import dataclass
from typing import List

import numpy as np

from ksb.motion.trajectories import TrajectoryProfile, P, V, A


@dataclass
class SegmentEvents:
    """Per-(input, segment) event times and endpoint kinematic states.

    Input index i ∈ {0, ..., b-1}.
    Segment index k ∈ {0, ..., N_B-1}: segment k occupies buffer-local [P^B_k, P^B_{k+1}].
    All times are absolute (global) time.
    """
    b:      int
    t_out:  np.ndarray  # (b, N_B)  t input i trailing edge clears segment k exit
    t_in:   np.ndarray  # (b, N_B)  t input i leading edge reaches segment k entry
    
    x_out:  np.ndarray  # (b, N_B, 3)  state x at time t_out
    x_in:   np.ndarray  # (b, N_B, 3)  state x at time t_in

    @property
    def W(self) -> np.ndarray:
        """Budget matrix: free-window width per (pair, segment). Shape (b-1, N_B).

        W[i, k] = t_in[i+1, k] - t_out[i, k]: time between leader i clearing
        segment k exit and follower i+1 reaching segment k entry.
        """
        return self.t_in[1:] - self.t_out[:-1]

    @property
    def a_out(self) -> np.ndarray:
        return self.x_out[:,:,A]

    @property
    def v_out(self) -> np.ndarray:
        return self.x_out[:,:,V]

    @property
    def p_out(self) -> np.ndarray:
        return self.x_out[:,:,P]

    @property
    def a_in(self) -> np.ndarray:
        return self.x_in[:,:,A]

    @property
    def v_in(self) -> np.ndarray:
        return self.x_in[:,:,V]

    @property
    def p_in(self) -> np.ndarray:
        return self.x_in[:,:,P]

def compute_segment_events(
    total_trajectories: List[TrajectoryProfile],
    t_spawn: np.ndarray,              # (b,)
    input_length: float,              # uniform l_i for now
    L_upstream: float,
    Ls: np.ndarray,                   # (N_B,) segment lengths summing to L_buffer
) -> SegmentEvents:
    """Compute per-(input, segment) event times and kinematic states.

    For each input i and buffer segment k, compute:
    - t_out[i, k]: absolute time when input i's trailing edge clears segment k exit
    - t_in[i, k]: absolute time when input i's leading edge reaches segment k entry
    - Kinematic states (v, a) at those instants

    Raises ValueError if any event's position target is unreachable in the composite trajectory.
    """
    b = len(total_trajectories)
    N_B = len(Ls)

    # Segment boundary positions in buffer-local coordinates from cumulative Ls
    boundaries = np.concatenate([[0.0], np.cumsum(Ls)])
    P_buffer_entries = boundaries[:N_B]
    P_buffer_exits = boundaries[1:]

    # Convert to absolute positions (composite trajectory frame)
    P_abs_entries = L_upstream + P_buffer_entries
    P_abs_exits = L_upstream + P_buffer_exits

    t_out = np.zeros((b, N_B))
    t_in = np.zeros((b, N_B))
    x_out = np.zeros((b, N_B, 3))
    # a_out = np.zeros((b, N_B))
    x_in = np.zeros((b, N_B, 3))
    # a_in = np.zeros((b, N_B))

    for i in range(b):
        traj = total_trajectories[i]

        for k in range(N_B):
            p_out = P_abs_exits[k] + input_length  # trailing edge at segment exit
            p_in = P_abs_entries[k]                 # leading edge at segment entry

            t_i_out = traj.find_time_at_position(p_out)
            if t_i_out is None:
                raise ValueError(
                    f"Input {i} cannot reach segment {k} exit position "
                    f"{p_out:.6f} m (unreachable in trajectory)"
                )

            t_i_in = traj.find_time_at_position(p_in)
            if t_i_in is None:
                raise ValueError(
                    f"Input {i} cannot reach segment {k} entry position "
                    f"{p_in:.6f} m (unreachable in trajectory)"
                )

            t_out[i, k] = t_spawn[i] + t_i_out
            t_in[i, k] = t_spawn[i] + t_i_in

            x_out[i, k] = traj.eval(t_i_out) 
            x_in[i,k] = traj.eval(t_i_in) 

    return SegmentEvents(
        b=b,
        t_out=t_out,
        t_in=t_in,
        x_out=x_out,
        x_in=x_in,
    )
