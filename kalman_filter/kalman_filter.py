import numpy as np

def kalman_filter(X_obs, A, C, Gamma, Sigma, mu_0, V_0):
    """
    Kalman filter forward pass (Bishop 13.89-13.92).
    
    Parameters
    ----------
    X_obs : np.ndarray, shape (N, d_obs)
        Sequence of observations.
    A : np.ndarray, shape (d_z, d_z)
        State transition matrix.
    C : np.ndarray, shape (d_obs, d_z)
        Emission matrix.
    Gamma : np.ndarray, shape (d_z, d_z)
        Process noise covariance.
    Sigma : np.ndarray, shape (d_obs, d_obs)
        Measurement noise covariance.
    mu_0 : np.ndarray, shape (d_z, 1)
        Initial state mean.
    V_0 : np.ndarray, shape (d_z, d_z)
        Initial state covariance.

    Returns
    -------
    mu_filtered : np.ndarray, shape (N, d_z)
    V_filtered  : np.ndarray, shape (N, d_z, d_z)
    """
    N = X_obs.shape[0]
    d_z = A.shape[0]
    I = np.eye(d_z)

    mu_n, V_n = mu_0, V_0
    mu_filtered, V_filtered = [], []

    for n in range(N):
        x_n = X_obs[n]

        # predict: uncertainty about z_n before seeing x_n
        P = A @ V_n @ A.T + Gamma

        # Kalman gain: how much to trust the observation vs the prediction
        K_n = P @ C.T @ np.linalg.inv(C @ P @ C.T + Sigma)

        # residual: how much x_n surprised us
        residual = K_n @ (x_n - C @ A @ mu_n)

        # update: correct prediction using the residual
        mu_n = A @ mu_n + residual
        V_n = (I - K_n @ C) @ P

        mu_filtered.append(mu_n)
        V_filtered.append(V_n)

    return np.array(mu_filtered), np.array(V_filtered)

def extrapolate(mu_n, V_n, A, Gamma, k):
    """
    Extrapolate k steps forward using only the dynamics model.
    No observations — predict step only.
    """
    mu_future, V_future = [], []
    
    for _ in range(k):
        mu_n = A @ mu_n
        V_n  = A @ V_n @ A.T + Gamma # NOTE: V_n grows monotonically!
        mu_future.append(mu_n)
        V_future.append(V_n)
    
    return np.array(mu_future), np.array(V_future)