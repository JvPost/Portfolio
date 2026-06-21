import numpy as np
import matplotlib.pyplot as plt
from BO import BO
from GP import GP
from scipy.stats import norm
from acquisition_functions import ei, ucb, thompson_sampling
from kernels import SquaredExponential
from utils import compute_cumulative_regret, compute_simple_regret

def homogeneous_blackbox(x):
    return np.sin(x) + np.sin((10.0 / 3.0) * x)

def heterogeneous_blackbox(x):
    # This part will give a high-frequency oscillation that decreases in amplitude as x increases.
    term1 = np.exp(-0.1 * x) * np.sin(5 * np.pi * x)
    
    # This part will add a linearly decreasing trend across the entire domain.
    term2 = -0.5 * x
    
    # This part will add a low-frequency oscillation with increasing amplitude as x increases.
    term3 = np.exp(0.1 * np.abs(x)) * np.sin(2 * np.pi * x)

    return term1 + term2 + term3

if __name__ == "__main__":
    f = heterogeneous_blackbox
    x1_domain = (-10., 10.)
    obs_noise_var = .3
    config = {
        'parameters': {
            "x1": x1_domain,
        },
        "obs_noise_var": obs_noise_var
    }
    N_pred = 100
    max_iter = 20
    gp = GP(parameters=config['parameters'], kernel=SquaredExponential(.5))
    bo = BO(gp, f, thompson_sampling, config)
    X_test = np.linspace(*x1_domain, N_pred) 
    X_min = X_test[f(X_test).argmin()]
    Y_min = f(X_test).min() 
    

    X, Y, Mu, Sigma = bo.run(max_iter)
    simple_regret = compute_simple_regret(f(X), Y_min)
    cumulative_regert = compute_cumulative_regret(f(X), Y_min) 

    print("Distance from actual noise var: ", np.abs(gp.noise_var - obs_noise_var))
    fig = plt.figure(figsize=(10, 5))
    ax1 = fig.add_subplot(121)
    ax1.set_title("Performance")  
    ax1.plot(simple_regret, label="Simple Regret")
    ax1.plot(cumulative_regert, label="Cumulative regret")
    ax1.legend()

    ax2 = fig.add_subplot(122)
    ax2.set_title("Samples using %s acquisition" % "UCB")  
    ax2.scatter(X, Y, color="blue", marker="+", label="Training Data")
    ax2.plot(X_test, Mu, color="blue", label="gpr")
    ax2.fill_between(X_test, Mu - 1.96 * np.diag(Sigma),
                     Mu + 1.96 * np.diag(Sigma), 
                     color="black", alpha=.1)

    ax2.plot(X_test, f(X_test), color="black", ls="--", label="Black box")
    ax2.legend()
    plt.show()
