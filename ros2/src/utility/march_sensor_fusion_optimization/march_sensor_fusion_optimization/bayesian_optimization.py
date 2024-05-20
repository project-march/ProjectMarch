"""
Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy
"""

from march_sensor_fusion_optimization.parameters_handler import ParametersHandler
from march_sensor_fusion_optimization.state_handler import BayesianOptimizationStates

import numpy as np
import tensorflow_probability as tfp
from tensorflow_probability import distributions as tfd
from tensorflow_probability import math as tfm

class BayesianOptimizer:

    def __init__(self, dof: int, max_iter: int, min_obs: float, param_file: str, amplitude=1.0, length_scale=1.0) -> None:
        """ Initialize the Bayesian Optimizer object.
        """
        assert dof > 2, "Degrees of freedom must be greater than 2."
        assert max_iter > 0, "Maximum number of iterations must be greater than 0."
        assert min_obs > 0, "Minimum observation change must be greater than 0."
        assert param_file is not None, "Parameter file must be provided."

        self.max_iter = max_iter
        self.min_obs = min_obs

        # Initialize parameter handler
        self.param_file = param_file
        self.parameter_handler = ParametersHandler(self.param_file)
        num_optimization_parameters = self.parameter_handler.get_num_optimization_parameters()

        # Initialize kernel hyperparameters
        self.amplitude  = tfp.util.TransformedVariable(
            amplitude, tfp.bijectors.Softplus(), dtype=np.float64, name='amplitude')
        self.length_scale = tfp.util.TransformedVariable(
            length_scale, tfp.bijectors.Softplus(), dtype=np.float64, name='length_scale')
        self.kernel = tfm.psd_kernels.ExponentiatedQuadratic(
            amplitude=self.amplitude, length_scale=self.length_scale)

        # Initialize surrogate model
        index_points = np.expand_dims(np.linspace(-1.0, 1.0, num_optimization_parameters), axis=-1)
        self.surrogate_model = tfd.StudentTProcess(
            df=float(dof),
            kernel=self.kernel,
            index_points=index_points,
        )


    def configure(self, population_size: int, max_time: float) -> None:
        """ Create random sets of observation noise.
        """
        assert population_size > 0, "Population size must be greater than 0."
        assert max_time > 0, "Maximum time must be greater than 0."
        self.population = self.surrogate_model.sample(population_size)
        self.population_performance = [None for _ in range(population_size)]
        self.optimizer_idx = -1
        self.max_time = max_time
        self.time = 0.0


    def run(self, idx: int, performance: float, timestep: float) -> bool:
        """ Store the performance cost of a set of observation noise.
        Update the time and check if the run has reached the maximum time.
        """
        assert idx >= 0, "Index must be greater than or equal to 0."
        assert performance is not None, "Performance must be provided."
        assert timestep > 0, "Timestep must be greater than 0."

        if self.population_performance[idx] is not None:
            self.population_performance[idx].append(performance)
        else:
            self.population_performance[idx] = [performance]
        
        # Update time and check if the run has reached the maximum time
        self.time += timestep
        return self.time >= self.max_time


    def fit(self) -> None:
        """run EKF system multiple times using each set of observation noise, get the JNIS
        performance cost of each set, and fit the surrogate model to the performance costs.
        """
        pass
        

    def optimize(self) -> None:
        """ Maximize the acquisition function to find the optimal set of observation noise.
        Use the optimal set of observation noise to run the EKF system and get the JNIS to
        update the surrogate model. Repeat until the surrogate model converges or the
        termination criterion is met.
        """
        pass

    
    def acquisition_func(self, samples: np.ndarray, optimizer: np.ndarray) -> np.ndarray:
        """ Calculate the acquisition function in order to intelligently guide the search
        for the optimal set of observation noise via the surrogate model.
        """
        pass