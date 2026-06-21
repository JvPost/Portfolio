import numpy as np

def compute_simple_regret(Y_train, Y_star):
    return Y_train - Y_star

def compute_cumulative_regret(Y_train, Y_min):
    Y_min_arr = np.ones_like(Y_train) * Y_min 
    Y_min_arr[0] = 0
    return np.cumsum(Y_train - Y_min_arr)