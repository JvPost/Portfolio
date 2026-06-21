import numpy as np
from abc import ABC, abstractmethod

class Kernel(ABC):
    def __init__(self, length_scales:np.ndarray, sigma=1.0):
        assert length_scales.ndim == 1
        self.l = length_scales
        self.s = sigma

    @abstractmethod
    def __call__(self, x1, x2):
        pass
        

class SquaredExponential(Kernel):
    """
    This class represents the Squared Exponential kernel (also known as Gaussian kernel), 
    which is a commonly used kernel function in Gaussian Process regression and Bayesian optimization.
    """
    def __init__(self, length_scale=1.0, sigma=1.0):
        """
        Initialize the Squared Exponential kernel with a given length scale parameter 'l'.
        
        Parameters:
        length_scale (float): The length scale parameter of the kernel. This controls the smoothness of the function.
        """
        super().__init__(length_scale, sigma)

    def __call__(self, x1, x2):
        """
        This method calculates the Squared Exponential kernel function between two points.

        The Squared Exponential kernel function is a measure of similarity between two points.
        It is a real-valued function that depends only on the distance between the points.
        The kernel function decreases as the distance between the points increases, 
        meaning that points that are closer together are considered more similar.

        Parameters:
        x1, x2: numpy arrays
            The points between which the kernel function is to be computed. They must be 
            of the same dimension.

        Returns:
        k: float
            The value of the Squared Exponential kernel function between x1 and x2.
        """
        diff = x1 - x2
        k = self.s**2 * np.exp(-0.5 * np.sum(diff**2 / self.l**2))
        return k

class Matern52(Kernel):
    def __init__(self, length_scale=1.0, sigma=1.0):
        super().__init__(length_scale, sigma)

    def __call__(self, x1, x2):
        # Calculate the Euclidean distance between x and y
        dist = np.linalg.norm(x1 - x2)
        
        # Calculate the Matérn 5/2 kernel value
        factor = np.sqrt(5) * dist / self.l
        matern_value = (1 + factor + (5/3) * (dist**2) / (self.l**2)) * np.exp(-factor)
        
        return self.s**2 * matern_value

class Periodic(Kernel):
    def __init__(self, length_scale=1.0, sigma=1.0, period=1.0):
        super().__init__(length_scale, sigma)
        self.p = period

    def __call__(self, x1, x2):
        dist = np.linalg.norm(x1 - x2)
        return self.s**2 * np.exp(-2 * np.sin(np.pi * dist / self.p)**2 / self.l**2)
