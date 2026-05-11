import numpy as np
import matplotlib.pyplot as plt

from ksb.simulation.result import SimulationResult
from ksb.motion.trajectories import (
    CompositeTrajectory,
    ConstantJerkTrajectory,
    LinearTrajectory,
    PolynomialTrajectory,
    P, V, A,
)


def _jerk_segment(seg, t_arr: np.ndarray):
    """Jerk for a *primitive* (non-composite) segment at local times t_arr."""
    if isinstance(seg, ConstantJerkTrajectory):
        return np.full(len(t_arr), seg.jerk)
    if isinstance(seg, LinearTrajectory):
        return np.zeros(len(t_arr))
    if isinstance(seg, PolynomialTrajectory):
        return seg.poly.deriv(3)(t_arr)
    return None


def _eval_jerk(traj, t_arr: np.ndarray):
    """Evaluate jerk for any TrajectoryProfile at times t_arr.

    For CompositeTrajectory, routes each sample to the correct segment and
    recurses, so nested composites are handled correctly.

    Returns ndarray shape (len(t_arr),), or None if an unknown segment type
    is encountered.
    """
    if not isinstance(traj, CompositeTrajectory):
        return _jerk_segment(traj, t_arr)

    cum_durs = np.cumsum([0.0] + [seg.T for seg in traj.segments])
    t_c = np.clip(t_arr, 0.0, traj.T)
    seg_indices = np.clip(
        np.searchsorted(cum_durs, t_c, side='right') - 1,
        0, len(traj.segments) - 1,
    )

    jerk_out = np.empty(len(t_arr))
    for seg_idx in np.unique(seg_indices):
        mask = seg_indices == seg_idx
        t_local = t_c[mask] - cum_durs[seg_idx]
        j = _eval_jerk(traj.segments[seg_idx], t_local)
        if j is None:
            return None
        jerk_out[mask] = j
    return jerk_out


class InputPlotter:
    """Plots per-input kinematics and gap curves for KSB simulation results.

    Encapsulates configuration and styling for consistent, reproducible plots.
    """

    def __init__(self, cfg: dict):
        """Initialize plotter with configuration.

        Parameters
        ----------
        cfg : dict
            Configuration dict containing at least:
            - 'Vmax': maximum velocity (m/s)
            - 'Amax': maximum acceleration (m/s²)
            - 'jmax': maximum jerk (m/s³)
            - 'L_buffer': buffer length (m)
            - 'n_buffer_seg': number of buffer segments (count, not boundaries)
        """
        self.cfg = cfg
        self.Vmax = float(cfg.get('Vmax', 3.0))
        self.Amax = float(cfg.get('Amax', 8.5))
        self.jmax = float(cfg.get('jmax', 100.0))
        self.L_buffer = float(cfg.get('L_buffer', 2.0))
        self.n_buffer_seg = int(cfg.get('n_buffer_seg', 5))

    def plot(self, result: SimulationResult, N: int, n_samples: int = 500, i_start: int = 0):
        """Plot kinematics and gap curves for the first N inputs.

        Parameters
        ----------
        result : SimulationResult
            Simulation result from KSBSimulation.run()
        N : int
            Number of input rows to plot
        n_samples : int
            Evaluation points per trajectory domain (default 500)
        i_start : int
            Starting input index (default 0)

        Returns
        -------
        fig : matplotlib.figure.Figure
            The resulting figure.

        Figure layout: N rows × 5 columns
          col 0 — position (m)         ┐
          col 1 — velocity (m/s)       │  buffer + straddle + registrar trajectory for input i
          col 2 — acceleration (m/s²)  │
          col 3 — jerk (m/s³)          ┘  [blank if unsupported by segment type]
          col 4 — g_i(t): gap between input i and input i+1 (m)
                   evaluated over exactly [t_window_start, t_window_end]
        """
        B = len(result.buffer_trajectories)
        assert N <= B, f"N={N} exceeds number of trajectories ({B})"

        COL_LABELS = [
            'Position (m)', 'Velocity (m/s)', 'Acceleration (m/s²)',
            'Jerk (m/s³)', r'Gap $g_i(t)$ (m)',
        ]

        fig, axes = plt.subplots(
            N, 5,
            figsize=(20, 3.2 * N),
            squeeze=False,
            constrained_layout=True,
        )
        fig.suptitle('KSB — per-input kinematics and gap curves', fontsize=14)

        for col, label in enumerate(COL_LABELS):
            axes[0, col].set_title(label, fontsize=11)

        for i in range(N):
            # Segments: [0]=upstream, [1]=buffer, [2]=straddle, [3]=registrar, [4]=downstream
            segs = result.system_trajectories[i + i_start].segments
            buff_traj = segs[1]
            straddle_traj = segs[2]
            reg_traj = segs[3]

            combined_traj = CompositeTrajectory(
                x0=buff_traj.x0,
                T=buff_traj.T + straddle_traj.T + reg_traj.T,
                segments=(buff_traj, straddle_traj, reg_traj),
            )

            # Boundary times within combined_traj for shading
            t_straddle_start = buff_traj.T
            t_registrar_start = buff_traj.T + straddle_traj.T

            # ── Kinematics ──────────────────────────────────────────────
            t = np.linspace(0.0, combined_traj.T, n_samples)
            states = combined_traj.eval(t)  # shape (3, n_samples)

            axes[i, 0].annotate(
                f'Input {i + i_start + 1}',
                xy=(-0.32, 0.5), xycoords='axes fraction',
                ha='center', va='center', fontsize=11, fontweight='bold', rotation=90,
            )

            def _shade(ax):
                """Shade straddle, registrar, and buffer regions."""
                ax.axvspan(t_registrar_start, combined_traj.T,
                           alpha=0.10, color='C3', label='registrar')
                self._shade_buffer(ax, buff_traj, t, states)

            # col 0 — position
            axes[i, 0].plot(t, states[P], color='C0')
            _shade(axes[i, 0])
            axes[i, 0].set_xlabel('t (s)')
            axes[i, 0].set_ylabel('p (m)')

            # col 1 — velocity
            axes[i, 1].plot(t, states[V], color='C1')
            _shade(axes[i, 1])
            axes[i, 1].set_xlabel('t (s)')
            axes[i, 1].set_ylabel('v (m/s)')
            axes[i, 1].set_ylim(0, self.Vmax)

            # col 2 — acceleration
            axes[i, 2].plot(t, states[A], color='C2')
            _shade(axes[i, 2])
            axes[i, 2].set_xlabel('t (s)')
            axes[i, 2].set_ylabel('a (m/s²)')
            axes[i, 2].set_ylim(-self.Amax, self.Amax)

            # col 3 — jerk
            jerk = _eval_jerk(combined_traj, t)
            if jerk is not None:
                axes[i, 3].plot(t, jerk, color='C3')
                _shade(axes[i, 3])
                axes[i, 3].set_xlabel('t (s)')
                axes[i, 3].set_ylabel('j (m/s³)')
            else:
                axes[i, 3].axis('off')
                axes[i, 3].text(
                    0.5, 0.5,
                    'Jerk not available\n(segment type has no\nanalytical jerk method)',
                    ha='center', va='center',
                    transform=axes[i, 3].transAxes,
                    fontsize=9, color='gray',
                )
            axes[i, 3].set_ylim(-self.jmax * 1.05, self.jmax * 1.05)

            # Add legend to first row only (shared meaning across rows)
            if i == 0:
                axes[i, 0].legend(fontsize=7, loc='upper left')

            # ── Gap g_i(t) ────────────────────────────────────────────────────────
            ax_gap = axes[i, 4]
            ax_gap.set_ylim(0, 2.0)

            if i >= B - 1:
                ax_gap.axis('off')
                ax_gap.text(0.5, 0.5, 'No follower\n(last input)',
                            ha='center', va='center',
                            transform=ax_gap.transAxes, fontsize=9, color='gray')
            else:
                pr = result.pair_records[i]

                t_follow = np.linspace(pr.t_start, pr.t_end, n_samples)
                t_lead = t_follow + pr.delta_t

                comp_lead = result.system_trajectories[i]
                comp_follow = result.system_trajectories[i + 1]

                p_lead = comp_lead.eval(t_lead)[P]
                p_follow = comp_follow.eval(t_follow)[P]
                gap = p_lead - p_follow

                t_offset = result.t_spawn[i + 1] - result.t_control_start[i]
                t_display = t_follow + t_offset

                ax_gap.plot(t_display, gap, color='C4', label=r'$g_i(t)$')
                ax_gap.set_xlabel('t (s, rel. to input i buffer entry)')
                ax_gap.set_ylabel('gap (m)')

                g_min = pr.g_min_threshold
                if g_min > 0.0:
                    ax_gap.axhline(
                        g_min, color='red', linestyle='--', linewidth=1.2,
                        label=fr'$g_{{\min}}$ = {g_min:.3f} m',
                    )
                ax_gap.legend(fontsize=8)

        plt.show()
        return fig

    def _shade_buffer(self, ax, buff_traj, t: np.ndarray, states: np.ndarray):
        """Shade buffer segment boundaries in kinematics axes.

        Parameters
        ----------
        ax : matplotlib.axes.Axes
            Axis to shade
        buff_traj : TrajectoryProfile
            Buffer trajectory segment
        t : np.ndarray
            Time array used for evaluation (shape (n_samples,))
        states : np.ndarray
            State array from combined_traj.eval(t) (shape (3, n_samples))
        """
        # Compute buffer boundary positions: 0, L/n, 2L/n, ..., L
        n_bounds = self.n_buffer_seg + 1
        boundaries = np.linspace(0, self.L_buffer, n_bounds)

        # Find times when position crosses each boundary
        t_boundaries = []
        for p_b in boundaries:
            idx = np.argmin(np.abs(states[P, :] - p_b))
            t_boundaries.append(t[idx])

        # Shade regions between boundaries with alternating alphas
        alphas = [0.06, 0.12]
        for j in range(len(t_boundaries) - 1):
            alpha = alphas[j % 2]
            ax.axvspan(t_boundaries[j], t_boundaries[j + 1],
                       alpha=alpha, color='C0', zorder=-1, 
                       label="buffer" if j == 0 else "")