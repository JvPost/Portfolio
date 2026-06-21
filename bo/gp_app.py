import numpy as np
import matplotlib.pyplot as plt
from GP import GP
from scipy.stats import norm
from kernels import SquaredExponential, Matern52, Periodic

def simple(x):
    return np.sin(2*np.pi*x).squeeze()

def simple_2d(x):
    x1, x2 = x[:, 0], x[:, 1]
    return (1 - x1)**2 + 100 * (x2 - x1**2)**2

def homogeneous_blackbox(x : np.ndarray) -> np.ndarray:
    y = np.sin(x) + np.sin((10.0 / 3.0) * x)
    return y.squeeze()

def heterogeneous_blackbox(x):
    # This part will give a high-frequency oscillation that decreases in amplitude as x increases.
    term1 = np.exp(-0.1 * x) * np.sin(5 * np.pi * x)
    
    # This part will add a linearly decreasing trend across the entire domain.
    term2 = -0.5 * x
    
    # This part will add a low-frequency oscillation with increasing amplitude as x increases.
    term3 = np.exp(0.1 * np.abs(x)) * np.sin(2 * np.pi * x)

    return term1 + term2 + term3

if __name__ == "__main__":
    seed = 42
    np.random.seed(seed)
    f = simple
    x1_domain = (-1, 1) 
    obs_noise_var = 1e-1 # aleatoric noise
    config = {
        'parameters': {
            "x1": x1_domain,
        },
        "obs_noise_var": obs_noise_var
    }
    N_pred = 100
    N_train = 20

    domains = np.array(list(config["parameters"].values()), dtype=np.float32)
    ndim = domains.shape[0]

    X = np.random.uniform(domains[:, 0], domains[:, 1], (N_train, ndim))
    Y = f(X) + np.random.normal(0, np.sqrt(obs_noise_var), N_train)
    X_test = np.linspace(domains[:, 0], domains[:, 1], N_pred) 

    initial_length_scales = np.array([0.25, 0.25])
    initial_signal_var = 1.

    kernel = SquaredExponential(initial_length_scales, initial_signal_var)

    gp = GP(parameters=config['parameters'], 
            kernel=kernel, 
            N_pred=N_pred, initial_noise_var=.1, jitter=0)

    Mu, Sigma, noise_var = gp.update_data_and_predict(X, Y)

    fig = plt.figure(figsize=(5, 5))
    ax = fig.add_subplot(111)
    ax.scatter(X, Y, color="blue", marker="+", label="Training Data")
    ax.plot(X_test, Mu, color="blue", label="$\mu$")
    ax.fill_between(X_test.squeeze(), 
                    Mu - 1.96 * Sigma,
                    Mu + 1.96 * Sigma, 
                    color="blue", alpha=.1, label="$\Sigma$"
                    )
    ax.plot(X_test, f(X_test), color="black", ls="--", label="Black box")
    ax.legend()
    print("Distance from actual noise var: ", np.abs(noise_var - obs_noise_var))
    print("Final noise variance: ", noise_var)

    print("Final length scale: ", gp.kernel.l)
    print("Final signal variance: ", gp.kernel.s)
    plt.show()
