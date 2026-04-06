from __future__ import annotations

from typing import List, Optional

import numpy as np

from ksb.control.upstream_control import ConstantVelocityControl, UpstreamController, PreAccelerateControl
from ksb.motion.item_pair import PairRecord, compute_pairs
from ksb.motion.trajectories import CompositeTrajectory, TrajectoryProfile, P, V, A
from ksb.planning.contracts import IProfileSolver, Policy
from ksb.planning.solvers.linear import LinearTrajectorySolver
from ksb.simulation.result import SimulationResult
import ksb.simulation.utils as utils


class KSBSimulation:
    """Kinematic Synchronization Buffer simulation.

    Receives stochastic item arrivals and plans jerk-limited trajectories to
    deliver each item to a fixed, deterministic slot schedule.
    """
    def __init__(self, cfg: dict, solver: IProfileSolver) -> None:
        self.cfg = cfg
        self.solver = solver

        self.jmax = float(cfg["jmax"])
        self.Vmax = float(cfg.get("Vmax", 3.0))
        self.Amax = float(cfg.get("Amax", 8.5))

        self.L_upstream = float(cfg.get("L_upstream", 1.0)) # the length of actual upstream
        self.L_buffer = float(cfg.get("L_buffer", 2.0))
        self.L_downstream = float(cfg.get("L_downstream", 1.0))

        self.input_length = float(cfg.get("input_length", 0.32))
        self.N = int(cfg.get("N", 5))

        Lmin_factor = float(cfg.get("Lmin_factor", 1.25))
        Lmin = Lmin_factor * self.input_length
        Ls = np.array(self.N * [self.L_buffer / self.N])
        assert np.all(Ls >= Lmin), \
            "All buffer sections must be at least Lmin_factor * input_length"

        self.min_gap_on_buffer = self.L_buffer / self.N + self.input_length

        self.slot_length = float(cfg.get("slot_length", 0.40))
        self.gap_mean = float(cfg.get("input_gap_mean", 0.80))
        self.gap_std = float(cfg.get("input_gap_std", 0.05))

        arrival_rate_ppm = float(cfg.get("arrival_rate_ppm", 180))
        slot_rate_ppm = float(cfg.get("slot_rate_ppm", 180))

        self.ru = arrival_rate_ppm / 60.0
        self.rd = slot_rate_ppm / 60.0
        assert self.rd >= self.ru, "slot rate must be >= arrival rate"

        self.vu = self.ru * self.gap_mean
        self.vd = self.rd * self.slot_length
        self.slot_period = self.slot_length / self.vd

        self.batch = int(cfg.get("batch", 5))

        start_margin = float(cfg.get("start_margin", 1.0))
        end_margin = float(cfg.get("end_margin", 0.0))

        assert end_margin == 0.0, "non zero end_margin not implemented yet"
        self.L_buffer_ctrl = self.L_buffer - self.input_length * start_margin # buffer control length

        self.L_upstream_ctrl = self.L_upstream + start_margin * self.input_length # The length over which usptream control active

        # bounds as numpy array: [j_max, A_max, V_max, gap_min]
        self.bounds = np.array([
            self.jmax,
            self.Amax,
            self.Vmax,
            self.min_gap_on_buffer,
        ])
        self.policy = Policy(input_length=self.input_length)

        # self._u_control = ConstantVelocityControl(self.vu)
        self._u_control = PreAccelerateControl(self.vu, self.jmax, self.Amax, 
                                               1.0, self.Vmax)
        self._d_solver = LinearTrajectorySolver()

    def run(self, seed: Optional[int] = None) -> SimulationResult:
        vu, vd = self.vu, self.vd
        L_upstream, L_downstream = self.L_upstream, self.L_downstream
        L_buffer_ctrl, L_upstream_ctrl = self.L_buffer_ctrl, self.L_upstream_ctrl
        slot_period = self.slot_period
        bounds, policy = self.bounds, self.policy
        
        x0_upstream = np.array([0.0, vu, 0.0])

        # 1) Spawn times
        t_spawn = utils.input_spawn_times_ar1(
            self.batch,
            v0=vu,
            mean=self.gap_mean,
            std=self.gap_std,
            min=self.input_length,
            seed=seed,
        )

        # 2) upstream control & 3) slot assignment
        assigned_slots = np.empty(self.batch, dtype=int)
        t_control_start = np.empty(self.batch, dtype=float)
        t_duration_upstream = np.empty(self.batch, dtype=float)
        buffer_trajectories:List[TrajectoryProfile] = []
        upstream_ctrl_trajectories:List[TrajectoryProfile] = []
        slot_idx = 0 # find a better heuristic than just 0

        prev_slot_idx = None
        
        for i, t0 in enumerate(t_spawn):
            upstream_ctrl_traj = self._u_control.subsection(t0, L_upstream_ctrl)

            t_in = t0 + upstream_ctrl_traj.T
            v_in = self._u_control._state_at(t_in)[V]

            slot_idx, buffer_traj = utils.get_next_slot(t_in, slot_idx, self.slot_length, 
                                            v_in, self.vd, L_buffer_ctrl, self.bounds, 
                                            self.policy, self.solver)
            
            if prev_slot_idx != None:
                skipped = slot_idx > prev_slot_idx + 1
                if skipped:
                    self._u_control.on_skip(t_in)
             
            buffer_trajectories.append(buffer_traj)

            t_control_start[i] = t_in
            assigned_slots[i] = slot_idx
            t_duration_upstream[i] = upstream_ctrl_traj.T
            upstream_ctrl_trajectories.append(upstream_ctrl_traj)

            prev_slot_idx = slot_idx

        assigned_slot_times = assigned_slots * slot_period
        buffer_T_array = assigned_slot_times - t_control_start

        # 4) Phase errors
        projected_no_corr = t_spawn + (L_upstream + self.L_buffer) / vu
        phi_u = (assigned_slot_times - projected_no_corr) / slot_period

        projected_no_corr_at_entry = t_control_start + L_buffer_ctrl / vu
        phi_0 = (projected_no_corr_at_entry - assigned_slot_times) / slot_period

        skip_indices = np.arange(1, self.batch)[np.diff(assigned_slots) > 1] - 1

        # 5) Buffer + downstream trajectories
        total_trajectories: List[CompositeTrajectory] = []
        downstream_T = L_downstream / vd

        for i, tf in enumerate(buffer_T_array):
            sb_traj = buffer_trajectories[i]
            d_traj = self._d_solver.solve(
                pi=0.0, vi=vd, pf=L_downstream, vf=vd, T=downstream_T,
                bounds=bounds, policy=policy,
            )
            total_T = t_duration_upstream[i] + tf + downstream_T
            comp_traj = CompositeTrajectory(
                x0=x0_upstream,
                T=total_T,
                segments=(upstream_ctrl_trajectories[i], sb_traj, d_traj),
            )
            total_trajectories.append(comp_traj)

        # 6) pair record 
        input_delta_t = np.diff(t_spawn)

        # t_window_start = np.array([t.T for t in upstream_trajectories])[1:]
        t_window_start = np.array([t.find_time_at_position(L_upstream) for t in upstream_ctrl_trajectories])[1:]

        t_window_end = t_window_start + buffer_T_array[:-1]

        pairs: List[PairRecord] = []
        if self.batch > 1:
            pairs = compute_pairs(
                trajectories=total_trajectories,
                delta_t=input_delta_t,
                t_rel_start=t_window_start,
                t_rel_end=t_window_end,
                n_points=1200,
            )

            for p in pairs:
                p.compute_integrals(g_min=self.min_gap_on_buffer)

        return SimulationResult(
            cfg=self.cfg,
            t_spawn=t_spawn,
            t_control_start=t_control_start,
            assigned_slots=assigned_slots,
            time_horizons=buffer_T_array,
            skip_indices=skip_indices,
            phi_u=phi_u,
            phi_0=phi_0,
            composite_trajectories=total_trajectories,
            buffer_trajectories=buffer_trajectories,
            pair_records=pairs,
        )
