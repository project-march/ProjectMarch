"""
Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy
"""

from march_sensor_fusion_optimization.parameters_handler import ParametersHandler
from march_sensor_fusion_optimization.state_handler import BayesianOptimizationStates

import random as rd
import numpy as np
import tensorflow_probability as tfp
from tensorflow_probability import distributions as tfd
from tensorflow_probability import math as tfm

class BayesianOptimizer:

    def __init__(self, dof: int, max_iter: int, min_obs: float, param_file: str, amplitude=1.0, length_scale=1.0, initial_population_size=10) -> None:
        """ Initialize the Bayesian Optimizer object.
        """
        np.random.seed(42)

        assert dof > 2, "Degrees of freedom must be greater than 2."
        assert max_iter > 0, "Maximum number of iterations must be greater than 0."
        assert min_obs > 0, "Minimum observation change must be greater than 0."
        assert param_file is not None, "Parameter file must be provided."

        self.max_iter = max_iter
        self.min_obs = min_obs

        # Initialize parameter handler
        self.param_file = param_file
        self.parameter_handler = ParametersHandler(self.param_file)
        self.num_optimization_parameters = self.parameter_handler.get_num_optimization_parameters()

        # Initialize kernel hyperparameters
        assert amplitude > 0, "Amplitude must be greater than 0."
        assert length_scale > 0, "Length scale must be greater than 0."
        self.amplitude  = tfp.util.TransformedVariable(
            amplitude, tfp.bijectors.Softplus(), dtype=np.float64, name='amplitude')
        self.length_scale = tfp.util.TransformedVariable(
            length_scale, tfp.bijectors.Softplus(), dtype=np.float64, name='length_scale')
        self.kernel = tfm.psd_kernels.ExponentiatedQuadratic(
            amplitude=self.amplitude, length_scale=self.length_scale, )

        # Create a random set of noise parameters.
        self.dof = float(dof)
        assert initial_population_size > 0, "Initial population size must be greater than 0."
        self.initial_population_size = initial_population_size

        # Initialize the samples
        self.X_sample = None
        self.y_sample = None

        # Initialize bounds for the optimization parameters
        self.bounds = np.array([(1e-12, 10.0) for _ in range(self.num_optimization_parameters)])

    def configure_noise_parameters(self, noise_parameters: np.ndarray) -> None:
        """ Configure the state estimator with the provided noise parameters.
        """
        assert noise_parameters is not None, "Noise parameters must be provided."
        assert len(noise_parameters) == self.num_optimization_parameters, \
            "Number of noise parameters {} must match the number of optimization parameters which is {}" \
            .format(len(noise_parameters),self.num_optimization_parameters)
        self.parameter_handler.set_optimization_parameters(noise_parameters.tolist())
        self.total_time = 0.0
        self.costs = []
        self.average_costs = []

    def select_random_noise_parameters(self) -> None:
        """ Select a random set of noise parameters from the initial population,
        and configure the state estimator with these parameters.
        """
        random_parameters = np.random.uniform(self.bounds[:, 0], self.bounds[:, 1], size=(self.num_optimization_parameters,))
        self.populate(random_parameters)            
        self.configure_noise_parameters(random_parameters)

    def collect_performance_cost(self, performance: float, dt: float) -> None:
        """ Collect the performance of the state estimator to the last set of noise parameters.
        """
        assert performance is not None, "Performance must be provided."
        assert dt > 0.0, "Time step must be greater than 0."
        self.costs.append(performance)
        self.total_time += dt

    def compute_average_performance_cost(self) -> None:
        """ Compute the average performance of the state estimator with the last set of noise parameters
        and reset the list of performance costs for the next Monte Carlo run.
        """
        assert len(self.X_sample) > 0, "X_sample must not be empty."
        assert len(self.costs) > 0, "Performance costs must not be empty."
        self.average_costs.append(np.mean(self.costs))
        self.costs = []
    
    def evaluate_performance_cost(self) -> None:
        """ Compute the JNIS performance cost of the state estimator with the current set of noise parameters.
        """
        assert len(self.X_sample) > 0, "Population performance must not be empty."
        assert len(self.average_costs) > 0, "Population performance must not be empty."
        assert self.total_time > 0.0, "Total time must be greater than 0."
        self.y_sample[-1, 0] = np.log(np.sum(self.average_costs) / (self.total_time * self.dof))

    def fit(self) -> None:
        """ Fit the surrogate model to the current population of noise parameters and JNIS performance costs.
        """
        assert len(self.X_sample) > 0, "Population must not be empty."
        assert len(self.X_sample[-1]) == self.num_optimization_parameters, \
            "Number of noise parameters {} must match the number of optimization parameters which is {}" \
            .format(len(self.X_sample[-1]),self.num_optimization_parameters)
        assert len(self.y_sample) > 0, "Performance costs must not be empty."
        self.surrogate_model = self.build_surrogate_model(self.X_sample)
        self.mu = self.surrogate_model.mean(index_points=self.X_sample).numpy().reshape(-1, 1)
        self.sigma = self.surrogate_model.stddev(index_points=self.X_sample).numpy().reshape(-1, 1)

    def select_acqusition_noise_parameters(self) -> None:
        """ Calculate the acquisition function in order to intelligently guide the search
        for the optimal set of observation noise via the surrogate model.
        """
        optimizer_idx = np.min([self.population[p]['JNIS'] for p in range(len(self.population))])

    def optimize(self) -> None:
        """ Maximize the acquisition function to find the optimal set of observation noise.
        Use the optimal set of observation noise to run the EKF system and get the JNIS to
        update the surrogate model. Repeat until the surrogate model converges or the
        termination criterion is met.
        """
        pass

    def populate(self, noise_parameters: np.ndarray) -> None:
        """ Populate the surrogate model with a set of provided noise parameters
        with the corresponding list of JNIS performance costs.
        """
        if self.X_sample is not None:
            self.X_sample = np.vstack([self.X_sample, noise_parameters])
            self.y_sample = np.vstack([self.y_sample, np.array((np.inf,))])
        else:
            self.X_sample = noise_parameters.reshape(1, -1)
            self.y_sample = np.array((np.inf)).reshape(1, 1)
    
    def get_expected_improvement(self, y_optimizer: np.ndarray) -> np.ndarray:
        """ Calculate the expected improvement of the surrogate model.
        """
        assert y_optimizer is not None, "Optimal performance cost must be provided."
        assert len(y_optimizer) == self.num_optimization_parameters, \
            "Number of optimal performance costs {} must match the number of optimization parameters which is {}" \
            .format(len(y_optimizer),self.num_optimization_parameters)
        z = (self.mu - y_optimizer) / self.sigma
        return (self.mu - y_optimizer) * tfd.Normal(0.0, 1.0).cdf(z) + self.sigma * tfd.Normal(0.0, 1.0).prob(z)

    def build_surrogate_model(self, X: np.ndarray) -> tfp.distributions.StudentTProcess:
        """ Return the surrogate model of the Bayesian Optimizer.
        """
        return tfp.distributions.StudentTProcess(
            df=self.dof,
            kernel=self.kernel,
            index_points=np.expand_dims(X.flatten(), axis=-1),
            observation_noise_variance=1e-6,
        )
    
