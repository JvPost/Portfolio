from __future__ import annotations

from dataclasses import dataclass
from typing import List

import numpy as np

from ksb.motion.trajectories import CompositeTrajectory, P, V, A


@dataclass
class SegmentEvents:
    """Per-(pair, segment) event times and endpoint kinematic states.

    Pair index i ∈ {0, ..., b-2}: leader = input i, follower = input i+1.
    Segment index k ∈ {0, ..., N_B-1}: segment k occupies buffer-local [P^B_k, P^B_{k+1}].
    All times are absolute (global) time.
    """
    t_out:    np.ndarray  # (b-1, N_B)  t leader trailing edge clears segment k exit
    t_in:     np.ndarray  # (b-1, N_B)  t follower leading edge reaches segment k entry
    v_minus:  np.ndarray  # (b-1, N_B)  v at t_out, from leader's trajectory
    a_minus:  np.ndarray  # (b-1, N_B)
    v_plus:   np.ndarray  # (b-1, N_B)  v at t_in, from follower's trajectory
    a_plus:   np.ndarray  # (b-1, N_B)

    @property
    def W(self) -> np.ndarray:
        """Budget matrix: free-window width per (pair, segment)."""
        return self.t_in - self.t_out


def compute_segment_events(
    total_trajectories: List[CompositeTrajectory],
    t_spawn: np.ndarray,              # (b,)
    input_length: float,              # uniform l_i for now
    L_upstream: float,
    Ls: np.ndarray,                   # (N_B,) segment lengths summing to L_buffer
) -> SegmentEvents:
    """Compute per-(pair, segment) event times and kinematic states.

    For each pair (i, i+1) and buffer segment k, compute:
    - t_out[i, k]: absolute time when leader i's trailing edge clears segment k exit
    - t_in[i, k]: absolute time when follower i+1's leading edge reaches segment k entry
    - Kinematic states (v, a) at those instants

    Raises ValueError if any event's position target is unreachable in the composite trajectory.
    """
    b = len(total_trajectories)
    n_pairs = b - 1
    N_B = len(Ls)

    # Segment boundary positions in buffer-local coordinates from cumulative Ls
    boundaries = np.concatenate([[0.0], np.cumsum(Ls)])
    P_buffer_entries = boundaries[:N_B]
    P_buffer_exits = boundaries[1:]

    # Convert to absolute positions (composite trajectory frame)
    P_abs_entries = L_upstream + P_buffer_entries
    P_abs_exits = L_upstream + P_buffer_exits

    # Initialize result arrays
    t_out = np.zeros((n_pairs, N_B))
    t_in = np.zeros((n_pairs, N_B))
    v_minus = np.zeros((n_pairs, N_B))
    a_minus = np.zeros((n_pairs, N_B))
    v_plus = np.zeros((n_pairs, N_B))
    a_plus = np.zeros((n_pairs, N_B))

    # Compute events for each pair and segment
    for i in range(n_pairs):
        traj_lead = total_trajectories[i]
        traj_follow = total_trajectories[i + 1]

        for k in range(N_B):
            # Position targets for this segment
            p_exit_with_length = P_abs_exits[k] + input_length  # leader trailing edge at segment exit
            p_entry = P_abs_entries[k]  # follower leading edge at segment entry

            # Find local times
            t_i_local = traj_lead.find_time_at_position(p_exit_with_length)
            if t_i_local is None:
                raise ValueError(
                    f"Leader {i} cannot reach segment {k} exit position "
                    f"{p_exit_with_length:.6f} m (unreachable in trajectory)"
                )

            t_f_local = traj_follow.find_time_at_position(p_entry)
            if t_f_local is None:
                raise ValueError(
                    f"Follower {i+1} cannot reach segment {k} entry position "
                    f"{p_entry:.6f} m (unreachable in trajectory)"
                )

            # Convert to absolute time
            t_out[i, k] = t_spawn[i] + t_i_local
            t_in[i, k] = t_spawn[i + 1] + t_f_local

            # Evaluate kinematic states
            state_minus = traj_lead.eval(t_i_local)  # shape (3,)
            state_plus = traj_follow.eval(t_f_local)  # shape (3,)

            v_minus[i, k] = state_minus[V]
            a_minus[i, k] = state_minus[A]
            v_plus[i, k] = state_plus[V]
            a_plus[i, k] = state_plus[A]

    return SegmentEvents(
        t_out=t_out,
        t_in=t_in,
        v_minus=v_minus,
        a_minus=a_minus,
        v_plus=v_plus,
        a_plus=a_plus,
    )
