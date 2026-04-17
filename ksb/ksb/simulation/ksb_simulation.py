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
    def __init__(self, cfg: dict) -> None:
        self.cfg = cfg
        self.solver_name = cfg.get("solver", "")
        self.solver = utils.get_solver_from_name(self.solver_name)

        self.jmax = float(cfg["jmax"])
        self.Vmax = float(cfg.get("Vmax", 3.0))
        self.Amax = float(cfg.get("Amax", 8.5))

        self.L_upstream = float(cfg.get("L_upstream", 1.0)) # the length of actual upstream
        self.L_buffer = float(cfg.get("L_buffer", 2.0))
        self.L_reg = float(cfg.get("L_registrar", 3.0))
        self.L_downstream = float(cfg.get("L_downstream", 1.0))

        self.input_length = float(cfg.get("input_length", 0.32))
        self.n_buffer_seg = int(cfg.get("n_buffer_seg", 5))
        self.n_reg_seg = int(cfg.get("n_reg_seg", 5))

        Lmin_factor = float(cfg.get("Lmin_factor", 1.25))
        Lmin = Lmin_factor * self.input_length
        Ls = np.array(self.n_buffer_seg * [self.L_buffer / self.n_buffer_seg])
        assert np.all(Ls >= Lmin), \
            "All buffer sections must be at least Lmin_factor * input_length"

        self.slot_length = float(cfg.get("slot_length", 0.40))
        self.gap_mean = float(cfg.get("input_gap_mean", 0.80))
        self.gap_std = float(cfg.get("input_gap_std", 0.05))
        # self.gap_min = self.L_buffer / self.n_buffer_seg + self.input_length
        self.gap_min = 2 * self.L_buffer / self.n_buffer_seg

        arrival_rate_ppm = float(cfg.get("arrival_rate_ppm", 180))
        slot_rate_ppm = float(cfg.get("slot_rate_ppm", 180))

        self.ru = arrival_rate_ppm / 60.0
        self.rd = slot_rate_ppm / 60.0
        assert self.rd >= self.ru, "slot rate must be >= arrival rate"

        self.vu = self.ru * self.gap_mean
        self.vd = self.rd * self.slot_length

        ### EXPERIMENTAL
        # TODO: This is a good variable to optimize over as well, when we chose to optimize the entire KSB system party won a supermajority.
        self.v_buff_out = float(cfg.get('v_buff_out', 2.0)) # v^{BR}
        assert self.v_buff_out > self.vd

        self.slot_period = self.slot_length / self.vd

        self.batch = int(cfg.get("batch", 5))

        start_margin = float(cfg.get("start_margin", 1.0))
        end_margin = float(cfg.get("end_margin", 0.0))

        assert end_margin == 0.0, "non zero end_margin not implemented yet"
        self.L_upstream_ctrl = self.L_upstream + (start_margin * self.input_length) # The length over which usptream control active
        self.L_buffer_ctrl = self.L_buffer - (self.input_length * start_margin) # buffer control length

        ### EXPERIMENTAL 
        # registry hack
        self.L_buffer_end_offset = ( self.L_buffer / self.n_buffer_seg - self.input_length ) 
        self.T_buffer_end_offset = self.L_buffer_end_offset / self.vd

        self.L_buffer_ctrl = self.L_buffer_ctrl - self.L_buffer_end_offset
        

        # bounds as numpy array: [j_max, A_max, V_max, gap_min]
        self.bounds = np.array([
            self.jmax,
            self.Amax,
            self.Vmax,
            self.gap_min,
        ])
        self.policy = Policy(input_length=self.input_length)

        # self._u_control = ConstantVelocityControl(self.vu)
        self._u_control = PreAccelerateControl(vu = self.vu, 
                                               j_max = self.jmax * 1.0, 
                                               a_max = self.Amax, 
                                               a_max_acc = 0.5,
                                            #    v_max_up= self.vu + (self.Vmax - self.vu) * .5
                                                v_max_up=2.
                                               )
                                               
        self._d_solver = LinearTrajectorySolver()

        

    def run(self, seed: Optional[int] = None) -> SimulationResult:
        vu, vd = self.vu, self.vd
        L_upstream, L_downstream = self.L_upstream, self.L_downstream
        L_buffer_ctrl, L_upstream_ctrl = self.L_buffer_ctrl, self.L_upstream_ctrl
        slot_period = self.slot_period
        bounds, policy = self.bounds, self.policy
        
        x0_upstream = np.array([0.0, vu, 0.0])

        # 1) Spawn times
        # t_spawn = utils.input_spawn_times(batch = self.batch, v0=vu, 
        #                                   mean=self.gap_mean, std=self.gap_std,
        #                                   min=self.input_length, seed=seed)

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
                                            self.policy, self.solver, 
                                            t_offset=-self.T_buffer_end_offset)
            
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
        buffer_T_array = assigned_slot_times - t_control_start - self.T_buffer_end_offset

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
            buffer_traj = buffer_trajectories[i]
            d_traj = self._d_solver.solve(
                pi=0.0, vi=vd, pf=L_downstream, vf=vd, T=downstream_T,
                bounds=bounds, policy=policy,
            )
            total_T = t_duration_upstream[i] + tf + downstream_T + self.T_buffer_end_offset

            r_traj = self._d_solver.solve(
                pi=.0, vi=vd, pf=self.L_buffer_end_offset, vf=vd, 
                T=self.T_buffer_end_offset, bounds=bounds, policy=policy
            )

            comp_traj = CompositeTrajectory(
                x0=x0_upstream,
                T=total_T,
                segments=(upstream_ctrl_trajectories[i], buffer_traj, r_traj, d_traj),
            )
            total_trajectories.append(comp_traj)

        # 6) pair record 
        p_window_start = L_upstream
        p_window_end = L_upstream + self.L_buffer + self.input_length
        

        t_i_boundary_upstream_buffer = np.array(
            [traj.find_time_at_position(p_window_start) for traj in total_trajectories]
            )

        input_delta_t = np.diff(t_spawn + t_i_boundary_upstream_buffer) 
        
        # Relative time windows for i and j = i+1. We start keeping track of time
        # when input j enters the buffer. 
        t_j_window_start = np.array(t_i_boundary_upstream_buffer[1:])

        # We stop keeping track of time when input i's trailing edge leaves the buffer (p_window_end)
        # Since local time start with input $j$ and ends with a position for input $i$,
        # we have to substract the relative time difference.
        t_j_window_end   = np.array(
            [traj.find_time_at_position(p_window_end) for traj in total_trajectories[:-1]]
        ) - input_delta_t - ((self.L_buffer / self.n_buffer_seg) / self.vd)

        # Check some of the logic. 
        assert np.all(t_j_window_end <= np.array([traj.T for traj in total_trajectories[1:]])), \
            "t_window_end exceeds follower trajectory duration — overtake detected"
        assert np.all(t_j_window_end > t_j_window_start), \
            "t_window_end <= t_window_start — degenerate or inverted window"

        pairs: List[PairRecord] = []
        if self.batch > 1:
            pairs = compute_pairs(
                trajectories=total_trajectories,
                delta_t=input_delta_t,
                t_rel_start=t_j_window_start,
                t_rel_end=t_j_window_end,
                n_points=1200,
            )

            for p in pairs:
                p.compute_integrals(g_min=self.gap_min)

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
