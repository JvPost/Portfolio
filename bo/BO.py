import numpy as np
import matplotlib.pyplot as plt
from GP import GP 

class BO:
    def __init__(self,
                 gp_instance : GP,
                 objective_function : callable,
                 acquisition_function : callable,
                 config: dict,
                 ):
        self.gp = gp_instance
        self.f = objective_function
        self.acq = acquisition_function
        self.config = config 
   

    def run(self, max_iter=100, N_test=100):
        domain = self.config['parameters']['x1']
        X = np.zeros((max_iter))
        Y = np.zeros((max_iter))
        Mu = Sigma = None
        X[0] = np.random.uniform(*domain) # Initialize
        Y[0] = self.f(X[0]) \
            + np.random.normal(0, np.sqrt(self.config['obs_noise_var']))

        candidates = np.linspace(*domain, N_test)
        
        for i in range(1, max_iter):
            self.gp.update_data(X_new=X[i-1:i], Y_new=Y[i-1:i]) 
            Mu, Sigma, _ = self.gp.predict()

            X[i] = self.acq(Mu, Sigma, candidates, Y[:i])
            # observation noise
            eps = np.random.normal(0, np.sqrt(self.config['obs_noise_var'])) 
            Y[i] = self.f(X[i]) + eps


        return X, Y, Mu, Sigma
         
     