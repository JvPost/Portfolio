# Kalman Filter

A Kalman filter forward pass and dynamics-only extrapolation step, implemented directly on
NumPy (Bishop 13.75-13.92) — no `filterpy`, `pykalman`, or similar. The point of the
project is the recursion itself: the predict/update cycle that fuses a linear-Gaussian
dynamics model with noisy observations, and the Kalman gain that automatically weighs one
against the other.

## Layout

- **`kalman_filter.py`** — `kalman_filter`, the forward filtering pass: at each step,
  predicts the next state from the dynamics model, computes the Kalman gain, and corrects
  the prediction with the observation residual. Returns the full history of filtered means
  and covariances. Also `extrapolate`, a predict-only rollout that propagates the last
  filtered state `k` steps forward through the dynamics model alone (no observations), for
  forecasting beyond the data.
- **`inference.ipynb`** — worked examples building up in state dimension: a 1D
  piecewise-constant signal, a 2D position/velocity tracker, and a 3D
  position/velocity/acceleration tracker, plus an extrapolation demo and a walkthrough of
  the `Gamma`/`Sigma` trade-off (process vs. measurement noise) that the Kalman gain
  balances at every step.

## Running it

```bash
git clone https://github.com/JvPost/Portfolio.git
cd Portfolio/kalman_filter
pip install -e ".[dev]"
jupyter notebook inference.ipynb
```

Or use it directly:

```python
from kalman_filter import kalman_filter, extrapolate

mu_filtered, V_filtered = kalman_filter(X_obs, A, C, Gamma, Sigma, mu_0, V_0)
mu_future, V_future = extrapolate(mu_filtered[-1].reshape(-1, 1), V_filtered[-1], A, Gamma, k=5)
```

### Requirements

Python 3.10+. NumPy installs automatically with `pip install -e .`. `pip install -e
".[dev]"` additionally pulls in Jupyter and Matplotlib for `inference.ipynb`.
