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

    def __init__(self, dof: int, max_iter: int, min_obs: float, param_file: str) -> None:
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

        # Initialize performance costs
        self.performance_costs = []

        # Initialize kernel hyperparameters
        self.amplitude  = tfp.util.TransformedVariable(
            100.0, tfp.bijectors.Softplus(), dtype=np.float64, name='amplitude')
        self.length_scale = tfp.util.TransformedVariable(
            1000.0, tfp.bijectors.Softplus(), dtype=np.float64, name='length_scale')
        self.kernel = tfm.psd_kernels.ExponentiatedQuadratic(
            amplitude=self.amplitude, length_scale=self.length_scale)

        # Initialize surrogate model
        index_points = np.expand_dims(np.linspace(-1.0, 1.0, num_optimization_parameters), axis=-1)
        self.surrogate_model = tfd.StudentTProcess(
            df=float(dof),
            kernel=self.kernel,
            index_points=index_points,
        )


    def fit(self, population_size: int) -> None:
        """ Create random sets of observation noise, run EKF system multiple times
        using each set of observation noise, get the JNIS performance cost of each
        set, and fit the surrogate model to the performance costs.
        """
        assert population_size > 0, "Population size must be greater than 0."

        self.population = self.surrogate_model.sample(population_size)
        for set in self.population:
            print(set)
            # Run EKF system
            # Get JNIS performance cost
            # self.performance_costs.append(cost)
        

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