from copy import copy
import matplotlib.pyplot as plt
import numpy as np

from ruckig import InputParameter, OutputParameter, Ruckig, Result


def run():

    ruckig = Ruckig(1, 0.01)

    inp = InputParameter(1)
    out = OutputParameter(1)

    inp.current_velocity = [2.0]
    inp.current_acceleration = [0.1]

    inp.target_velocity = [2.5]
    inp.target_acceleration = [0.0]

    inp.max_velocity = [3.0]
    inp.max_acceleration = [8.5]
    inp.max_jerk = [10]

    inp.minimum_duration = 1.3

    out.pass_to_input(inp)
    

    first_output, out_list = None, []
    states = []
    res = Result.Working

    while res == Result.Working:
        res = ruckig.update(inp, out)

        print('\t'.join([f'{out.time:0.3f}'] + [f'{p:0.3f}' for p in out.new_position]))

        states.append([out.new_position, out.new_velocity, out.new_acceleration, out.new_jerk])

        out_list.append(copy(out))

        out.pass_to_input(inp)

        if not first_output:
            first_output = copy(out)

    print(f'Calculation duration: {first_output.calculation_duration:0.1f} [µs]')
    print(f'Trajectory duration: {first_output.trajectory.duration:0.4f} [s]')
    print(f"Achievable with current max jerk: {first_output.trajectory.duration <= inp.minimum_duration}")

    result = ruckig.calculate(inp, first_output.trajectory) # offline, don't really know how to use this though.
    states = np.array(states).squeeze()

    plt.plot(states, label=["p", "v", "a", "j"])
    plt.legend()
    plt.show()


if __name__ == "__main__":
    run()