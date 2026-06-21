import numpy as np
import matplotlib.pyplot as plt
from GP import GP
from kernels import SquaredExponential

def rosenbrock(X : np.ndarray) -> np.ndarray:
    x1, x2 = X[:, 0], X[:, 1]
    y : np.ndarray = (1 - x1)**2 + 100 * (x2 - x1**2)**2
    return y.squeeze()

def linear(X : np.ndarray) -> np.ndarray:
    x1, x2 = X[:, 0], X[:, 1]
    return x1+x2

def quadratic(X : np.ndarray) -> np.ndarray:
    x1, x2 = X[:, 0], X[:, 1]
    return x1**2 + x2**2

def gaussian_bump(X: np.ndarray) -> np.ndarray:
    return np.exp(-quadratic(X))
    

if __name__ == "__main__":
    seed = 42
    np.random.seed(seed)

    f = gaussian_bump
    obs_noise_var = 1e-1
    config = {
        'parameters': {
            "x1": (-1, 1),
            "x2": (-1, 1),
        },
        "obs_noise_var": obs_noise_var
    }

    N_pred = 10  # per dimension, so 50x50 grid
    N_train = 75

    domains = np.array(list(config["parameters"].values()), dtype=np.float32)
    ndim = domains.shape[0]

    # training data
    X = np.random.uniform(domains[:, 0], domains[:, 1], (N_train, ndim))
    Y = f(X) + np.random.normal(0, np.sqrt(obs_noise_var), N_train)

    kernel = SquaredExponential(np.ones(ndim), sigma=1.0)
    gp = GP(parameters=config['parameters'],
            kernel=kernel,
            N_pred=N_pred,
            initial_noise_var=0.1,
            jitter=1e-6)

    Mu, Sigma, noise_var = gp.update_data_and_predict(X, Y)

    print("Final length scales:", gp.kernel.l)
    print("Final signal variance:", gp.kernel.s)
    print("Final noise variance:", noise_var)

    # build 2D grid for plotting
    x1_grid = np.linspace(*config['parameters']['x1'], N_pred)
    x2_grid = np.linspace(*config['parameters']['x2'], N_pred)
    X1, X2 = np.meshgrid(x1_grid, x2_grid)

    # true f
    X_grid = np.column_stack([X1.ravel(), X2.ravel()])
    Y_true = f(X_grid).reshape(N_pred, N_pred)

    # GP mean is already on the X_pred grid (N_pred x N_pred flattened)
    Mu_grid = Mu.reshape(N_pred, N_pred)
    Sigma_std = np.sqrt(np.maximum(Sigma, 0)).reshape(N_pred, N_pred)

    # plot
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))

    # true function
    levels_true = np.linspace(Y_true.min(), Y_true.max(), 30)
    c0 = axes[0].contourf(X1, X2, Y_true, levels=levels_true, cmap='viridis')
    axes[0].scatter(X[:, 0], X[:, 1], c='red', marker='+', s=80, label='Training data')
    plt.colorbar(c0, ax=axes[0])
    axes[0].set_title(f'{f.__name__} (true)')
    axes[0].set_xlabel('x1')
    axes[0].set_ylabel('x2')
    axes[0].legend()

    # GP predictive mean
    levels_mu = np.linspace(Mu_grid.min(), Mu_grid.max(), 30)
    c1 = axes[1].contourf(X1, X2, Mu_grid, levels=levels_mu, cmap='viridis')
    axes[1].scatter(X[:, 0], X[:, 1], c='red', marker='+', s=80, label='Training data')
    plt.colorbar(c1, ax=axes[1])
    axes[1].set_title('GP predictive mean')
    axes[1].set_xlabel('x1')
    axes[1].set_ylabel('x2')
    axes[1].legend()

    # GP uncertainty
    c2 = axes[2].contourf(X1, X2, Sigma_std, cmap='plasma')
    axes[2].scatter(X[:, 0], X[:, 1], c='red', marker='+', s=80, label='Training data')
    plt.colorbar(c2, ax=axes[2])
    axes[2].set_title('GP std (epistemic uncertainty)')
    axes[2].set_xlabel('x1')
    axes[2].set_ylabel('x2')
    axes[2].legend()

    plt.tight_layout()
    plt.show()