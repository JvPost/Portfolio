from ruckig import Ruckig, InputParameter, Trajectory, ControlInterface
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    otg  = Ruckig(1)             # no control-cycle dt needed for pure offline
    inp  = InputParameter(1)     # set BCs and limits; do not set minimum_duration
    
    inp.control_interface = ControlInterface.Velocity

    inp.current_position = [0.0]
    inp.current_velocity = [2.0]
    inp.current_acceleration = [0.1]

    inp.target_velocity = [3.1]
    inp.target_acceleration = [8.6]
    

    # inp.max_velocity = [3.0]
    # inp.max_acceleration = [8.5]
    inp.max_jerk = [100]

    traj = Trajectory(1)

    result = otg.calculate(inp, traj)   # populates `traj` in one shot
    T_min  = traj.duration              # this is what you feed your feasibility check

    T = np.arange(0, T_min, 0.001)

    X = []    
    for t in T:
        p, v, a = traj.at_time(t)
        X.append([p, v, a])
    X = np.array(X).squeeze()
    plt.plot(T, X)
    plt.show()