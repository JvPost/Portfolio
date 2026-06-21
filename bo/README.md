# BO

A Gaussian process (GP) regressor and a sequential Bayesian optimization (BO) loop, both
implemented directly on NumPy/SciPy — no GPyTorch, BoTorch, or scikit-learn. The point of
the project is the machinery itself: building the Gram matrix, fitting kernel
hyperparameters by maximizing the log marginal likelihood, and using the resulting
posterior to drive an acquisition function toward the optimum of a black-box function.

## Layout

- **`GP.py`** — the `GP` class. Builds the Gram matrix from a kernel, fits the kernel's
  length scale(s), signal variance, and observation noise by maximizing the log marginal
  likelihood (L-BFGS-B over log-transformed hyperparameters, Cholesky-based for numerical
  stability), and predicts a posterior mean/variance over a fixed grid of test points.
  Generalizes beyond 1D — `parameters` is a dict of named variables, each with its own
  domain, and the grid/Gram bookkeeping scales with `ndim`.
- **`kernels.py`** — a `Kernel` ABC plus `SquaredExponential`, `Matern52`, and `Periodic`
  implementations.
- **`acquisition_functions.py`** — `thompson_sampling`, `ucb`, and `ei` (expected
  improvement), each mapping a GP posterior and the observed history to the next point to
  query.
- **`BO.py`** — the `BO` class: the loop that ties a `GP`, an objective function, and an
  acquisition function together, alternately observing the objective and refitting the GP.
- **`utils.py`** — simple and cumulative regret, for scoring a BO run against the true
  optimum.
- **`gp_app.py`** — standalone demo: fits a 1D GP to noisy samples of a sine wave and
  plots the recovered mean/uncertainty band against ground truth.
- **`gp_app2d.py`** — the 2D counterpart, fitting a GP to a 2D surface (Rosenbrock,
  Gaussian bump, etc.) and plotting the true surface, posterior mean, and posterior
  uncertainty side by side.
- **`bo_app.py`** — runs the full BO loop against a 1D black-box function and plots the
  sampled points against the true function along with simple/cumulative regret.
- **`bo_notebook.ipynb`** — the original scratch notebook the modules above were
  extracted from; kept for reference.

## Running it

```bash
git clone https://github.com/JvPost/Portfolio.git
cd Portfolio/bo
pip install -e .
python gp_app.py      # 1D GP regression demo
python gp_app2d.py    # 2D GP regression demo
python bo_app.py      # full Bayesian optimization loop
```

Each script is self-contained and ends in a `plt.show()`, so a display is required (or
swap in a non-interactive Matplotlib backend).

### Requirements

Python 3.10+. NumPy, SciPy, and Matplotlib install automatically with `pip install -e .`.
`pip install -e ".[dev]"` additionally pulls in Jupyter for `bo_notebook.ipynb`.
