import numpy as np
from scipy.stats import norm


def thompson_sampling(mu, sigma, X, Y_train):
    pred_dist = np.random.multivariate_normal(mu, sigma)
    return X[pred_dist.argmin()]

def ucb(mu, sigma, X, Y_train, kappa=1.96):
    std = np.sqrt(np.diag(sigma)) if sigma.ndim > 1 else np.sqrt(sigma)
    ucb_values = mu - kappa * std
    return X[np.argmin(ucb_values)]

def ei(mu, sigma, X, Y_train, xi=.01):
    Y_min = np.min(Y_train) # current best position found
    std = np.sqrt(np.diag(sigma)) # standard deviation from the diagonal of the covariance matrix

    with np.errstate(divide='ignore'):
        Delta_n = Y_min - mu - xi
        Z = Delta_n / std
        ei = (Delta_n) * norm.cdf(Z) + std * norm.pdf(Z)
        ei[std ==.0] = .0

    return [ei.argmax()]