import numpy as np
import matplotlib.pyplot as plt
from typing import Callable
import scipy.linalg
from scipy.optimize import minimize
from kernels import Kernel 

# TODO: make it handle unnormalized data more cleanly
class GP:
    """
    Gaussian Process (GP) class for Bayesian optimization.
    
    This class is designed to handle one-dimensional optimization problems using
    Gaussian Processes. It is equipped with methods for updating the GP with new
    data and evaluating the GP to predict means and variances over a set of test points.
    
    Attributes:
        noise_var (float): Data noise \sigma_n^2.
        kernel (callable): Kernel function defining the covariance between points in the GP.
        X (np.ndarray): Array of input data points for the GP.
        Y (np.ndarray): Array of output data corresponding to the input data points.
        K (np.ndarray): Covariance matrix of the GP constructed from input data points.
        K_star (np.ndarray): Covariance matrix between test points and training points.
        N_pred (int): Number of points in predictive distribution.
        X_pred (np.ndarray): Array of test points where the GP is evaluated.
        K_star_star (np.ndarray): Covariance matrix of the test points.
    """
    def __init__(self, 
                 parameters: dict,
                 kernel : Kernel,
                 N_pred: int = 100,
                 initial_noise_var = 1.,
                 jitter = 1e-6,
                 ):
        """
        Initializes the Gaussian Process with the given parameters.

        Args:
            parameters (dict): Dictionary with the range of the single variable for the GP.
            kernel (callable): The kernel function to compute covariances between points.
            N_pred (int): The number of test points for the GP, defaults to 100.
            initial_noise_var (float): initial guess for the noise variance.
        """
        self.noise_var = initial_noise_var
        
        # The kernel function used for computing covariances between points.
        self.kernel = kernel
        
        # Initialization of the GP data, which will be set upon receiving data.
        self.X = None  # Input points will be stored here.
        self.Y = None  # Output points will be stored here.
        
        # The covariance matrix for the input data and test points will be computed later.
        self.K = None  # Covariance matrix for input data.
        self.K_star = None  # Covariance matrix between test points and input data.
        
        # Number of test points and their actual values.
        domains = np.array(list(parameters.values()), dtype=np.float32)
        self.ndim = domains.shape[0]
        grids = [np.linspace(domains[i, 0], domains[i, 1], N_pred) for i in range(self.ndim)]
        mesh = np.meshgrid(*grids)
        self.X_pred = np.column_stack([m.ravel() for m in mesh])
        self.N_pred = self.X_pred.shape[0]  # now N_pred^ndim

        # Covariance matrix for the test points, which only depends on X_pred.
        self.K_star_star = self.__gram(self.X_pred)

        self.jitter = jitter 

    def __gram(self, X:np.ndarray):
        """
        Computes the Gram matrix (covariance matrix) for the given input points using the GP's kernel function.

        The Gram matrix is a symmetric matrix where the element at the i-th row and j-th column represents
        the covariance between the i-th and j-th points according to the kernel function of the GP. For a
        valid covariance function, this matrix is positive semi-definite.

        Args:
            X (np.ndarray): A one-dimensional numpy array of input points for which the Gram matrix is computed.

        Returns:
            np.ndarray: The computed symmetric Gram matrix with shape (N, N) where N is the number of input points.
        """
        N = X.shape[0]
        K = np.zeros((N, N))
        for n in range(N):
            for m in range(n+1):
                k = self.kernel(X[n], X[m])
                K[n, m] = k
                K[m, n] = k
                
        return K
    
    def __update_gram(self, X_new:np.ndarray):
        """
        Updates the Gram matrix with new input points. The Gram matrix is the covariances between the data, according
        to some kernel.

        This method expands the existing Gram matrix to include new input data points. It calculates the covariance
        values between all new points and existing points as well as the covariance amongst the new points themselves
        using the GP's kernel function, and integrates these values into the existing Gram matrix.

        Args:
            X_new (np.ndarray): A one-dimensional numpy array of new input points to be added to the GP.

        Returns:
            np.ndarray: The updated Gram matrix including the covariances of the new input points, 
                        with shape (N_old + N_new, N_old + N_new), where N_old is the number of 
                        existing input points and N_new is the number of new input points.
        """
        # Determine the number of new and old input points.
        N_new = X_new.shape[0]
        N_old = self.X.shape[0] - N_new if self.X is not None else 0
        
        # Create a new Gram matrix that can accommodate the old and new points.
        K_new = np.zeros((N_old + N_new, N_old + N_new))
        
        # If there is an existing Gram matrix, copy its values into the new matrix.
        if self.K is not None:
            K_new[:N_old, :N_old] = self.K
        
        # Calculate the covariance values for the new points.
        for n in range(N_old, N_old + N_new):
            for m in range(n + 1):  # Only compute the lower triangle and diagonal.
                k = self.kernel(self.X[n], self.X[m])
                K_new[n, m] = k
                K_new[m, n] = k  # Reflect to fill the upper triangle.

        return K_new

    def __update_k_star(self, X_new):
        """
        Updates the covariance matrix between the test points and training 
        points (K_star) with new training data.

        This method computes the covariance values between the test points and 
        new training points using the GP's kernel function. It then extends the 
        existing K_star matrix with these new covariance values. If K_star is
        not previously initialized (i.e., this is the first set of training 
        points), it will be created.

        Args:
            X_new (np.ndarray): A one-dimensional numpy array of new training input points.
            N_train_old (int): The number of training points before adding the new data.

        Side effects:
            Modifies the K_star attribute of the GP class by appending the new 
            covariance values to the existing matrix.
        """
        K_star_new = np.zeros((self.N_pred, X_new.shape[0]))
        for n in range(self.N_pred):
            for m in range(X_new.shape[0]):
                K_star_new[n, m] = self.kernel(self.X_pred[n], X_new[m])

        if self.K_star is not None:
            self.K_star = np.hstack((self.K_star, K_star_new))
        else:
            self.K_star = K_star_new


    def __log_marginal_likelihood(self, log_params) -> None:
        """
        Calculates the log marginal likelihood of the Gaussian Process model given log-transformed hyperparameters.

        This method updates the kernel length scale, signal variance, and noise variance to their exponentiated values.
        It then computes the Gram matrix for the current training inputs and calculates the log marginal likelihood
        using the Cholesky decomposition for numerical stability. If the Cholesky decomposition fails due to a non-positive
        definite matrix, it returns negative infinity, indicating a failure in computation or unsuitable hyperparameters.

        Args:
            log_l (float): Log-transformed length scale of the kernel.
            log_s (float): Log-transformed signal variance of the kernel.
            log_noise_var (float): Log-transformed noise variance of the model.

        Returns:
            float: The log marginal likelihood of the model given the hyperparameters. Returns -np.inf if the Cholesky
                   decomposition fails, indicating unsuitable hyperparameters.

        Side effects:
            Modifies the kernel parameters (length scale and signal variance) and noise variance of the model to their
            exponentiated values.
        """
        self.kernel.l = np.exp(log_params[:self.ndim])
        self.kernel.s = np.exp(log_params[self.ndim])
        self.noise_var = np.exp(log_params[self.ndim+1])
        
        self.K = self.__gram(self.X)
        n = self.X.shape[0]
        C = self.K + (self.noise_var + self.jitter) * np.eye(n) 
        
        try:
            L = np.linalg.cholesky(C)
            alpha = scipy.linalg.cho_solve((L, True), self.Y)
        except np.linalg.LinAlgError:
            return -np.inf

        log_det = 2 * np.sum(np.log(np.diag(L)))
        marginal_log_likelihood = -0.5 * \
            (np.dot(self.Y.T, alpha) + log_det + n * np.log(2 * np.pi))
        return marginal_log_likelihood

    def __optimize_hyperparameters(self) -> None:
        """
        Optimizes the hyperparameters of the Gaussian Process model by maximizing the log marginal likelihood.

        This method uses the L-BFGS-B optimization algorithm to find the log-transformed hyperparameters (length scale,
        signal variance, and noise variance) that maximize the log marginal likelihood of the model. The optimization
        is performed in the log space to ensure that the hyperparameters remain positive. Upon finding the optimal
        log-transformed hyperparameters, it updates the model's kernel length scale, signal variance, and noise variance
        to their exponentiated optimal values.

        Side effects:
            Modifies the kernel parameters (length scale and signal variance) and noise variance of the model to their
            optimized values.

        Note:
            This method does not return any value. It directly updates the hyperparameters of the model.
        """
        def objective(log_params):
            return -self.__log_marginal_likelihood(log_params)

        best_result = None
        best_log_likelihood = -np.inf

        # 10 random restarts
        for _ in range(1):
            initial_log_params = np.concatenate([
                np.random.uniform(np.log(1e-3), np.log(10), self.ndim), # length scales
                np.random.uniform(np.log(1e-3), np.log(10), 2), # noises
            ])

            result = minimize(objective, initial_log_params, method='L-BFGS-B',
                              bounds=[(np.log(1e-2), np.log(1e2))] * (self.ndim + 2))

            if -result.fun > best_log_likelihood:
                best_log_likelihood = -result.fun
                best_result = result
            
        self.kernel.l = np.exp(best_result.x[:self.ndim])
        self.kernel.s = np.exp(best_result.x[self.ndim])
        self.noise_var = np.exp(best_result.x[self.ndim + 1])

        
    def update_data(self, X_new, Y_new):
        """
        Updates the GP model with new training data. 
        
        This involves updating the input data points (X), output values (Y), the Gram matrix (K), 
        and the covariance matrix between test points and training points (K_star). If no prior data 
        exists, it initializes these attributes with the new data. Otherwise, it appends the new data 
        to the existing ones and updates the matrices accordingly.

        Args:
            X_new (np.ndarray or list): New input data points to be added to the GP. Can be a list, a 1D array, 
                                        or a 2D array with a shape of (n_samples, n_features).
            Y_new (np.ndarray or list): Corresponding output values to X_new. Can be a list or a 1D array.

        Side effects:
            Modifies the internal state of the GP, including the input data points (X), output values (Y),
            the Gram matrix (K), and the covariance matrix (K_star), to include information from the new data.
        """
        if X_new.ndim == 1:
            X_new = X_new[:, None]

        if self.X is None:
            self.X = X_new
            self.Y_raw = Y_new
        else:
            self.X = np.vstack((self.X, X_new))
            self.Y_raw = np.append(self.Y_raw, Y_new)

        # normalize Y
        self.Y_mean = self.Y_raw.mean()
        self.Y_std = self.Y_raw.std() if self.Y_raw.std() > 0 else 1.0
        self.Y = (self.Y_raw - self.Y_mean) / self.Y_std

        self.K = self.__gram(self.X)
        self.__update_k_star(X_new)

    def __update_k_star_full(self):
        self.K_star = np.zeros((self.N_pred, self.X.shape[0]))
        for n in range(self.N_pred):
            for m in range(self.X.shape[0]):
                self.K_star[n, m] = self.kernel(self.X_pred[n], self.X[m])

    def predict(self) :
        """
        Evaluates the Gaussian Process (GP) model to predict the mean and variance of the function at the test points.

        This method calculates the posterior mean and covariance matrix for the GP given the current training data.
        The posterior mean vector (Mu) provides a point estimate of the function at each test point, while the 
        diagonal of the posterior covariance matrix (Sigma) gives the variance at each test point, representing 
        the model's uncertainty.

        Returns:
            tuple:
                - Mu (np.ndarray): The posterior mean vector of the GP at the test points, with shape (N_pred,).
                - Sigma (np.ndarray): The posterior covariance matrix of the GP at the test points, with shape (N_pred, N_pred).

        Raises:
            ValueError: If the GP model has not been initialized with any data (X is None).
        """
        if self.X is None or self.K is None:
            raise ValueError("GP model has not been initialized with data.")

        self.__optimize_hyperparameters()

        self.K = self.__gram(self.X)
        self.__update_k_star_full()

        n = self.X.shape[0]
        C = self.K + np.eye(n) * (self.noise_var + self.jitter)
        L = scipy.linalg.cholesky(C, lower=True)

        alpha = scipy.linalg.cho_solve((L, True), self.Y)
        Mu_norm = self.K_star @ alpha

        K_star_star_diag = np.array([self.kernel(self.X_pred[i], self.X_pred[i])
                                        for i in range(self.N_pred)])
        v = scipy.linalg.cho_solve((L, True), self.K_star.T)
        Sigma_diag = K_star_star_diag - np.sum(self.K_star * v.T, axis=1)

        Mu = Mu_norm * self.Y_std + self.Y_mean
        Sigma_diag = Sigma_diag * self.Y_std**2

        return Mu, Sigma_diag, self.noise_var

    def update_data_and_predict(self, 
                                X : np.array, 
                                Y : np.array) -> tuple[np.array, np.array, float]:
        self.update_data(X, Y)
        return self.predict()