from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np
from scipy.special import ndtr, ndtri

from ksb.motion.trajectories import TrajectoryProfile
from ksb.planning.contracts import IProfileSolver, InfeasibleError, SlotAssignmentError, Policy
from ksb.planning.solvers.quintic import QuinticSolver
from ksb.planning.solvers.scurve import SCurveSolver


def input_spawn_times(
    batch: int,
    v0: float,
    mean: float,
    std: float,
    min: float,
    seed: Optional[int] = None,
) -> np.ndarray:
    """Vectorized spawn times on the upstream conveyor.

    Args:
        batch: number of items
        v0: infeed velocity (m/s)
        mean: gap mean (m)
        std: gap std (m)
        min: minimum gap (m)
        seed: random seed

    Returns:
        shape (batch,), absolute spawn times (s)
    """
    if batch <= 0:
        return np.empty(0, dtype=np.float32)
    if batch == 1:
        return np.array([0.0], dtype=np.float32)
    std_safe = np.max([1e-6, std])

    rng = np.random.default_rng(seed)
    v0_safe = float(v0) if v0 > 0.0 else 1e-3
    p_low = ndtr((min - mean) / std_safe)
    u = rng.uniform(p_low, 1.0, size=batch - 1)
    gaps = mean + std_safe * ndtri(u)
    inter_times = gaps / v0_safe
    return np.r_[0.0, np.cumsum(inter_times)].astype(np.float32)


def input_spawn_times_ar1(
    batch: int,
    v0: float,
    mean: float = 0.60,
    std: float = 0.05,
    phi: float = 0.5,
    min: float = 0.30,
    seed: int = 42,
) -> np.ndarray:
    """Generate absolute spawn times using an AR(1) process in log-space.

    The gap above the floor, g_i = gap_i - min_gap, is modeled as:
        y_i = mu_y + phi * (y_{i-1} - mu_y) + epsilon_i
        g_i = exp(y_i)
        gap_i = min_gap + g_i

    Args:
        batch: number of items
        v0: infeed velocity (m/s)
        mean_gap: target mean gap (m); must be > min_gap
        target_std: desired marginal std of gapes (m)
        phi: AR(1) autocorrelation coefficient (0 < phi < 1)
        min_gap: hard lower bound on gap
        seed: random seed

    Returns:
        shape (batch,), absolute spawn times (s)
    """
    if batch <= 0:
        return np.empty(0, dtype=np.float32)
    if batch == 1:
        return np.array([0.0], dtype=np.float32)

    m = mean - min
    if m <= 0:
        raise ValueError("mean_gap must be > min_gap")

    sigma_y2 = np.log(1.0 + (std / m) ** 2)
    sigma_y = np.sqrt(sigma_y2)
    mu_y = np.log(m) - sigma_y2 / 2.0
    sigma_e = sigma_y * np.sqrt(1.0 - phi ** 2)

    rng = np.random.default_rng(seed)
    v0_safe = max(float(v0), 1e-4)

    epsilon = rng.normal(0.0, sigma_e, size=batch)
    y = np.empty(batch, dtype=np.float64)
    y[0] = mu_y + epsilon[0]
    for i in range(1, batch):
        y[i] = mu_y + phi * (y[i - 1] - mu_y) + epsilon[i]

    gaps = min + np.exp(y)
    inter_times = gaps[:-1] / v0_safe
    return np.r_[0.0, np.cumsum(inter_times)].astype(np.float32)


def input_spawn_times_lognormal(
    batch: int,
    v0: float,
    input_length: float,
    mu: float,
    sigma: float,
    seed: Optional[int] = None,
) -> np.ndarray:
    """Spawn times where gap follows a lognormal, shifted by input_length.

    (gap - input_length) ~ LogNormal(mu_ln, sigma_ln) parameterized so
    that E[gap] = mu and Std[gap] = sigma. Guarantees gap > input_length.

    Args:
        batch: number of items
        v0: infeed velocity (m/s)
        input_length: input length (m)
        mu: desired mean gap (m); must be > input_length
        sigma: desired std of gap (m)
        seed: random seed

    Returns:
        shape (batch,), absolute spawn times (s)
    """
    if batch <= 0:
        return np.empty(0, dtype=np.float32)
    if batch == 1:
        return np.array([0.0], dtype=np.float32)

    m = mu - input_length
    if m <= 0:
        raise ValueError("mu must be > input_length")

    mu_ln = np.log(m ** 2 / np.sqrt(m ** 2 + sigma ** 2))
    sigma_ln = np.sqrt(np.log(1 + sigma ** 2 / m ** 2))

    rng = np.random.default_rng(seed)
    v0_safe = max(float(v0), 1e-4)

    gaps = rng.lognormal(mu_ln, sigma_ln, size=batch - 1)
    gaps = input_length + gaps
    inter_times = gaps / v0_safe

    return np.r_[0.0, np.cumsum(inter_times)].astype(np.float32)


def slot_spawn_times(
    count: int,
    v_slot: float,
    slot_length: float,
    start_time: float = 0,
    *,
    phase: float = 0.5,
    offset_m: float = 0.0,
) -> np.ndarray:
    """Absolute times when slot k hits the reference point.

    t_k = start_time + phase*To + offset_m/v + k*To, k=0...count-1
    where To = slot_length / v_slot.

    Args:
        count: number of slots
        v_slot: slot velocity (m/s)
        slot_length: slot spacing (m)
        start_time: reference start time (s)
        phase: fractional offset within one period (0.5 = slot centres)
        offset_m: additional position offset (m)

    Returns:
        shape (count,), absolute slot times (s)
    """
    if count <= 0:
        return np.empty(0, dtype=float)
    v_safe = v_slot if v_slot > 0.0 else 1e-9
    To = slot_length / v_safe
    t0 = start_time
    return t0 + To * np.arange(count, dtype=float)


def belt_lengths(
    N: int,
    L_total: float,
    L_min: float,
    beta: float = 0.0,
    gamma: float = 0.0,
) -> np.ndarray:
    """Generate N belt lengths summing to L_total, each >= L_min.
    Shaped by a log-quadratic softmax over centered, normalized indices.
    """
    if L_total < N * L_min:
        raise ValueError("Infeasible: L_total < N * L_min")
    R = L_total - N * L_min
    # centered, normalized indices: ~[-1, 1] for N >= 2
    k = (np.arange(1, N + 1) - (N + 1) / 2.0) / max((N - 1) / 2.0, 1.0)
    w = np.exp(beta * k + gamma * (k ** 2))
    w_sum = w.sum()
    if not np.isfinite(w_sum) or w_sum <= 0:
        raise ValueError("Numerical issue: weights collapsed.")
    p = w / w_sum
    return L_min + R * p


def get_assigned_slots(
    T0: np.ndarray,
    slot_length: float,
    vi: float,
    vf: float,
    L_buffer_ctrl: float,
    bounds: np.ndarray,
    policy: Policy,
    solver: IProfileSolver,
    t_offset: float = 0.0,
) -> Tuple[np.ndarray, List[TrajectoryProfile]]:
    """Assign each item to the earliest feasible slot index.

    For each item's control-start time T0[i], iterates over slot indices k
    until a time horizon T = k*slot_period - T0[i] allows the solver to
    produce a feasible trajectory covering L_buffer_ctrl metres in time T.

    Args:
        T0: control-start times, shape (batch,)
        slot_length: slot spacing (m)
        vi: initial velocity (m/s)
        vf: target velocity at slot (m/s)
        L_buffer_ctrl: distance to cover in the buffer (m)
        bounds: np.array([j_max, A_max, V_max, gap_min])
        policy: Policy config
        solver: trajectory solver
        t_offset: time offset for slot phase (s)

    Returns:
        slot_indices: shape (batch,), integer slot indices
    """
    item_slot_indeces = np.zeros_like(T0, dtype=int)
    item_slot_trajs = []
    vd = vf
    slot_period = slot_length / vd
    slot_idx = int((T0[0] + L_buffer_ctrl / vi) // slot_period) - 1 # minus one, because zero indexing

    i = 0
    for t0 in T0: # loop over inputs 
        found = False
        input_attempts = 0
        while not found:
            input_attempts += 1
            if input_attempts > 10:
                raise SlotAssignmentError(f"No feasible slot for item at t0={t0:.4f}")
            
            try:
                slot_idx += 1
                slot_time = slot_idx * slot_period + t_offset
                time_horizon = slot_time - t0
                traj = solver.solve(0.0, vi, L_buffer_ctrl, vd, time_horizon, bounds, policy)
                found = True
                item_slot_indeces[i] = slot_idx

                i += 1
            except InfeasibleError:
                continue

        item_slot_trajs.append(traj)

        if not found:
            raise ValueError(f"No feasible slot for item at t0={t0:.4f}")

    assert i == T0.shape[0]
    assert len(item_slot_indeces) == len(item_slot_trajs)
    return item_slot_indeces, item_slot_trajs

def get_next_slot(
    t_control_start: float,
    slot_idx: int,
    slot_length: float,
    vi: float,
    vf: float,
    L_buffer_ctrl: float,
    bounds: np.ndarray,
    policy: Policy,
    solver: IProfileSolver,
    t_offset: float = 0.0,
    vd_slot: Optional[float] = None,
) -> Tuple[int, TrajectoryProfile]:
    """Assign a single input to the earliest feasible slot index.

    Args:
        t_control_start: buffer entry time for this input (s)
        slot_idx:        slot index to start searching from
        slot_length:     slot spacing (m)
        vi:              initial velocity (m/s)
        vf:              target velocity at buffer exit (m/s)
        L_buffer_ctrl:   distance to cover in the buffer (m)
        bounds:          np.array([j_max, A_max, V_max, gap_min])
        policy:          Policy config
        solver:          trajectory solver
        t_offset:        time offset added to each slot time (s); use
                         -T_registrar to account for registrar transit
        vd_slot:         downstream speed used for slot period calculation
                         (m/s); defaults to vf when omitted, for the case
                         where the buffer exits directly at v_d

    Returns:
        slot_idx: assigned slot index
        traj:     feasible trajectory profile
    """
    slot_period = slot_length / (vd_slot if vd_slot is not None else vf)
    attempts = 0

    while True:
        attempts += 1
        if attempts > 20:
            raise SlotAssignmentError(
                f"No feasible slot for input at t_control_start={t_control_start:.4f}"
            )

        slot_idx += 1
        slot_time = slot_idx * slot_period + t_offset
        time_horizon = slot_time - t_control_start

        try:
            traj: TrajectoryProfile = solver.solve(0.0, vi, L_buffer_ctrl, vf, time_horizon, bounds, policy)
            return slot_idx, traj
        except InfeasibleError:
            continue

def get_solver_from_name(n) -> IProfileSolver:
    if n == 'quintic':
        return QuinticSolver()
    elif n == 'scurve':
        return SCurveSolver()
    else:
        raise NameError("unknown solver")