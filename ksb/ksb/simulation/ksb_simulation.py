from __future__ import annotations

from typing import List, Optional

import numpy as np

from ksb.analysis.events import compute_segment_events
from ksb.analysis.sync_response import SegmentSyncResponse
from ksb.control.registrar import RegistrarProfile
from ksb.control.upstream_control import ConstantVelocityControl, UpstreamController, PreAccelerateControl
from ksb.motion.item_pair import PairRecord, compute_pairs
from ksb.motion.trajectories import CompositeTrajectory, ConstantJerkTrajectory, LinearTrajectory, TrajectoryProfile, P, V, A
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

        self.L_upstream = float(cfg.get("L_upstream", 1.0))
        self.L_buffer = float(cfg.get("L_buffer", 2.0))
        self.L_reg = float(cfg.get("L_registrar", 3.0))
        self.L_downstream = float(cfg.get("L_downstream", 1.0))

        self.input_length = float(cfg.get("input_length", 0.32))
        self.n_buffer_seg = int(cfg.get("n_buffer_seg", 5))
        self.n_reg_seg = int(cfg.get("n_reg_seg", 5))

        Lmin_factor = float(cfg.get("Lmin_factor", 0.5))
        Lmin = Lmin_factor * self.input_length
        beta = float(cfg.get("beta", 0.0))
        gamma = float(cfg.get("gamma", 0.0))

        self.Ls = utils.belt_lengths(self.n_buffer_seg, self.L_buffer, Lmin, beta, gamma)

        eta_s = float(cfg.get("eta_s", 1.25))
        self.slot_length = eta_s * self.input_length
        self.gap_mean = float(cfg.get("input_gap_mean", 0.80))
        self.gap_std = float(cfg.get("input_gap_std", 0.05))
        self.gap_rho = float(cfg.get("input_gap_rho", 0.0))

        self.gap_min = 2 * self.L_buffer / self.n_buffer_seg

        arrival_rate_ppm = float(cfg.get("arrival_rate_ppm", 180))
        eta_r = float(cfg.get("eta_r", 1.0))

        self.ru = arrival_rate_ppm / 60.0
        self.rd = eta_r * self.ru
        assert self.rd >= self.ru, "slot rate must be >= arrival rate"

        self.vu = self.ru * self.gap_mean
        self.vd = self.rd * self.slot_length

        rate_delta = (self.rd - self.ru)
        self.Q = self.rd / rate_delta if rate_delta != 0 else float('inf')

        eta_v = float(cfg.get("eta_v", 1.16))  # kinematic headroom factor
        self.v_BR = eta_v * self.vd            # v^{BR} = eta_v * v_d

        _fp_atol = 1e-9  # tolerance for floating-point rounding at the constraint boundary

        assert self.v_BR >= self.vd - _fp_atol, \
            f"v_buff_out ({self.v_BR}) must be >= v_d ({self.vd}); eta_v >= 1"
        assert self.v_BR <= self.Vmax + _fp_atol, \
            f"v_buff_out ({self.v_BR}) exceeds Vmax ({self.Vmax}); reduce eta_v or eta_r/eta_s"

        self.slot_period = self.slot_length / self.vd

        self.batch = int(cfg.get("batch", 5))

        start_margin = float(cfg.get("start_margin", 1.0))
        end_margin = float(cfg.get("end_margin", 0.0))
        assert end_margin == 0.0, "non zero end_margin not implemented yet"

        # bounds as numpy array: [j_max, A_max, V_max, gap_min]
        self.input_bounds = np.array([
            self.jmax,
            self.Amax,
            self.Vmax,
        ])

        v_min = float(cfg.get("v_min", 0.0))
        self.policy = Policy(input_length=self.input_length, v_min=v_min)

        self.j_u_max = self.jmax
        self.v_u_max = self.Vmax
        self.a_u_max = np.abs(self.v_u_max - self.vu) / (self.Q * self.slot_period)

        upstream_control = cfg.get('upstream_control', 'acc')
        if (upstream_control == "acc"):
            self._u_control = PreAccelerateControl(vu=self.vu, 
                                                j_u_max=self.j_u_max,
                                                a_max=self.Amax, 
                                                a_u_max=self.a_u_max,
                                                v_u_max=self.v_u_max,
                                                )
        elif (upstream_control == "constant"):
            self._u_control = ConstantVelocityControl(self.vu)
        elif (upstream_control == "const"):
            self._u_control = ConstantVelocityControl(self.vu)
        else:
            raise KeyError("Unknown upstream control")
                                               
        self._d_solver = LinearTrajectorySolver()

    def run(self, seed: Optional[int] = None, *, skip_pair_records: bool = True) -> SimulationResult:
        vu, vd = self.vu, self.vd
        L_upstream, L_downstream = self.L_upstream, self.L_downstream
        slot_period = self.slot_period
        bounds, policy = self.input_bounds, self.policy
        
        x0_upstream = np.array([0.0, vu, 0.0])

        # 1) Spawn times
        batch_t_spawn = utils.input_spawn_times_ar1(
            self.batch,
            v0=vu,
            mean=self.gap_mean,
            std=self.gap_std,
            rho=self.gap_rho,
            min=self.input_length,
            seed=seed,
        )

        # 2) upstream control & 3) slot assignment
        assigned_slots = np.empty(self.batch, dtype=int)
        abs_t_buffer_start = np.empty(self.batch, dtype=float)
        buffer_T_array = np.empty_like(abs_t_buffer_start, dtype=float)

        upstream_trajectories:List[TrajectoryProfile] = []
        buffer_trajectories:List[TrajectoryProfile] = []
        registrar_trajectories:List[TrajectoryProfile] = []

        slot_idx = 0
        prev_slot_idx = None
        
        # Upstream control & slot assignments, which also computes buffer trajectory.
        L_upstream_traj = L_upstream + self.input_length
        L_buffer_traj = self.L_buffer - self.input_length

        phase_error_buffer = np.empty(self.batch, dtype=float)
        for i, t0 in enumerate(batch_t_spawn):
            upstream_traj:TrajectoryProfile = self._u_control.subsection(t0, L_upstream_traj)

            t_start_buffer_traj = t0 + upstream_traj.T 

            buffer_xi = upstream_traj.xf

            # buffer_vf = np.clip(buffer_xi[V], 0, self.Vmax) # equalize incoming and outgoing vel
            buffer_vf = self.v_BR # fixed

            buffer_af = .0

            _reg = RegistrarProfile(
                v_in=buffer_vf,
                v_out=self.vd,
                L_reg=self.L_reg,
                input_length=self.input_length,
                j_max=self.jmax,
                a_max=self.Amax
            )
            
            # duration if all we did is cruise
            T_no_corr = (self.L_buffer - self.input_length) / buffer_xi[V]

            #            straddle time                           registrar time
            t_offset = -((self.input_length / buffer_vf) + _reg.T_total)

            # compute buffer trajectories
            slot_idx, buffer_traj = utils.get_next_slot(
                i, t_start_buffer_traj, slot_idx, self.slot_length, buffer_xi[V], 
                buffer_vf, self.a_u_max, L_buffer_traj, self.input_bounds, 
                self.policy, self.solver, t_offset=t_offset, vd_slot=self.vd)

            phase_error_buffer[i] = (T_no_corr - buffer_traj.T) / slot_period
            
            if prev_slot_idx != None:
                skipped = slot_idx > prev_slot_idx + 1
                if skipped:
                    self._u_control.on_skip(t_start_buffer_traj)

            abs_t_buffer_start[i] = t_start_buffer_traj
            assigned_slots[i] = slot_idx

            upstream_trajectories.append(upstream_traj)
            buffer_trajectories.append(buffer_traj)
            registrar_trajectories.append(_reg.trajectory)

            prev_slot_idx = slot_idx

        assigned_slot_times = assigned_slots * slot_period

        # 4) Phase errors
        T_no_corr = batch_t_spawn + (L_upstream + self.L_buffer) / vu
        phase_error_upstream = (assigned_slot_times - T_no_corr) / slot_period

        # 5) downstream trajectories & construction of the entire history
        total_trajectories: List[CompositeTrajectory] = []
        downstream_T = L_downstream / vd

        for i in range(self.batch):
            T_buffer = buffer_trajectories[i].T
            buffer_T_array[i] = T_buffer

            T_straddle_BR = self.input_length / buffer_trajectories[i].xf[V]
            straddle_traj_BR = LinearTrajectory(
                x0=np.array([0.0, buffer_trajectories[i].xf[V], 0.0]),
                T=T_straddle_BR,
            )

            reg_traj = registrar_trajectories[i]

            total_T = (
                upstream_trajectories[i].T 
                + T_buffer + T_straddle_BR # buffer + straddle
                + reg_traj.T + downstream_T # after buffer
            )

            d_traj = self._d_solver.solve(
                pi=0.0, vi=vd, pf=L_downstream, vf=vd, T=downstream_T,
                bounds=bounds, policy=policy,
            )

            comp_traj = CompositeTrajectory(
                x0=x0_upstream,
                T=total_T,
                segments=(
                    upstream_trajectories[i],
                    buffer_trajectories[i],
                    straddle_traj_BR,
                    reg_traj,
                    d_traj,
                ),
            )
            
            total_trajectories.append(comp_traj)

        #
        # 6) segment events
        #
        segment_events = None
        segment_sync_response = None
        if self.batch > 1:
            segment_events = compute_segment_events(
                total_trajectories=total_trajectories,
                t_spawn=batch_t_spawn,
                input_length=self.input_length,
                L_upstream=L_upstream,
                Ls=self.Ls,
            )

            a_max_sync = self.cfg['a_max_sync']
            j_max_sync = self.cfg['j_max_sync']
            bounds = np.array([a_max_sync, j_max_sync])
            segment_sync_response = SegmentSyncResponse(segment_events, bounds)

        skip_indices = np.arange(1, self.batch)[np.diff(assigned_slots) > 1] - 1

        return SimulationResult(
            cfg=self.cfg,
            t_spawn=batch_t_spawn,
            t_control_start=abs_t_buffer_start,
            assigned_slots=assigned_slots,
            time_horizons=buffer_T_array,
            skip_indices=skip_indices,
            phi_u=phase_error_upstream,
            phi_b=phase_error_buffer,
            system_trajectories=total_trajectories,
            buffer_trajectories=buffer_trajectories,
            segment_events=segment_events,
            segment_sync_response=segment_sync_response,
        )
