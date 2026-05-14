"""
Sanity check: how does Ruckig's position interface behave under minimum_duration?

Compares the time-optimal trajectory against a stretched one (2x duration).
- Outcome (a): stretched trajectory front-loads acceleration, then cruises at v_target.
- Outcome (b): stretched trajectory cruises at v_entry, then accelerates late.

(a) is what we want for pi^I. (b) would force us to scale bounds manually.
"""

from ruckig import Ruckig, InputParameter, Trajectory, ControlInterface
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    # Representative pi^I scenario from configs/system/default.yaml
    L_buffer = 3.0
    v_entry  = 1.9   # ~ upstream velocity
    v_BR     = 2.85  # eta_v * v_d, slot-aligned target
    v_max    = 3.0
    a_max    = 8.5   # pi^I bound (zero-slip)
    j_max    = 100.0

    def solve(min_duration=None):
        otg = Ruckig(1)
        inp = InputParameter(1)
        inp.control_interface = ControlInterface.Position

        inp.current_position     = [0.0]
        inp.current_velocity     = [v_entry]
        inp.current_acceleration = [0.0]

        inp.target_position      = [L_buffer]
        inp.target_velocity      = [v_BR]
        inp.target_acceleration  = [0.0]

        inp.max_velocity     = [v_max]
        inp.max_acceleration = [a_max]
        inp.max_jerk         = [j_max]

        if min_duration is not None:
            inp.minimum_duration = min_duration

        traj = Trajectory(1)
        otg.calculate(inp, traj)
        return traj

    # Solve time-optimal
    traj_opt = solve()
    T_opt = traj_opt.duration
    print(f"T_opt = {T_opt:.4f} s")

    # Solve with 2x stretched duration
    factor = 1.25
    T_stretched = factor * T_opt
    traj_stretched = solve(min_duration=T_stretched)
    print(f"T_stretched = {traj_stretched.duration:.4f} s (requested {T_stretched:.4f})")

    def sample(traj, dt=0.001):
        T = np.arange(0, traj.duration, dt)
        P, V, A = [], [], []
        for t in T:
            p, v, a = traj.at_time(t)
            P.append(p[0]); V.append(v[0]); A.append(a[0])
        return T, np.array(P), np.array(V), np.array(A)

    T1, P1, V1, A1 = sample(traj_opt)
    T2, P2, V2, A2 = sample(traj_stretched)

    fig, axes = plt.subplots(3, 2, figsize=(12, 9), sharex='col')

    axes[0, 0].plot(T1, V1); axes[0, 0].set_ylabel("v (m/s)")
    axes[0, 0].set_title(f"Time-optimal (T={T_opt:.3f}s)")
    axes[0, 0].axhline(v_BR, color='gray', ls='--', lw=0.5)
    axes[0, 0].axhline(v_entry, color='gray', ls='--', lw=0.5)

    axes[1, 0].plot(T1, A1, color='green'); axes[1, 0].set_ylabel("a (m/s²)")
    axes[1, 0].axhline(a_max, color='gray', ls='--', lw=0.5)

    axes[2, 0].plot(T1, P1, color='orange'); axes[2, 0].set_ylabel("p (m)")
    axes[2, 0].set_xlabel("t (s)")

    axes[0, 1].plot(T2, V2); axes[0, 1].set_title(f"Stretched (T={traj_stretched.duration:.3f}s)")
    axes[0, 1].axhline(v_BR, color='gray', ls='--', lw=0.5)
    axes[0, 1].axhline(v_entry, color='gray', ls='--', lw=0.5)

    axes[1, 1].plot(T2, A2, color='green')
    axes[1, 1].axhline(a_max, color='gray', ls='--', lw=0.5)

    axes[2, 1].plot(T2, P2, color='orange')
    axes[2, 1].set_xlabel("t (s)")

    fig.suptitle(f"Ruckig position interface: time-optimal vs minimum_duration={factor}*T_opt")
    plt.tight_layout()
    plt.show()