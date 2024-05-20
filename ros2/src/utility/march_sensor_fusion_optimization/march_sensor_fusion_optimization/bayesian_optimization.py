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
            amplitude=self.amplitude, length_scale=self.length_scale)

        # Create a random set of noise parameters.
        self.dof = float(dof)
        assert initial_population_size > 0, "Initial population size must be greater than 0."
        # self.initial_population = self.surrogate_model.sample(initial_population_size).numpy().tolist()
        self.initial_population = [np.random.uniform(0.0, 10.0, self.num_optimization_parameters).tolist() for _ in range(initial_population_size)]
        self.population = []
        self.index_points = np.array(self.initial_population).T

    def configure_noise_parameters(self, noise_parameters: list) -> None:
        """ Configure the state estimator with the provided noise parameters.
        """
        assert noise_parameters is not None, "Noise parameters must be provided."
        assert len(noise_parameters) == self.num_optimization_parameters, \
            "Number of noise parameters {} must match the number of optimization parameters which is {}" \
            .format(len(noise_parameters),self.num_optimization_parameters)
        self.parameter_handler.set_optimization_parameters(noise_parameters)
        self.total_time = 0.0
        self.costs = []
        self.average_costs = []

    def select_random_noise_parameters(self) -> None:
        """ Select a random set of noise parameters from the initial population,
        and configure the state estimator with these parameters.
        """
        random_idx = rd.randint(0, len(self.initial_population) - 1)
        self.populate(self.initial_population.pop(random_idx))            
        self.configure_noise_parameters(self.population[-1]['parameters'])

    def collect_performance(self, performance: float, dt: float) -> None:
        """ Collect the performance of the state estimator to the last set of noise parameters.
        """
        assert performance is not None, "Performance must be provided."
        assert dt > 0.0, "Time step must be greater than 0."
        self.costs.append(performance)
        self.total_time += dt

    def compute_average_performance(self) -> None:
        """ Compute the average performance of the state estimator with the last set of noise parameters
        and reset the list of performance costs for the next Monte Carlo run.
        """
        assert len(self.population) > 0, "Population performance must not be empty."
        assert len(self.population[-1]) > 0, "Population performance must not be empty."
        self.average_costs.append(np.mean(self.costs))
        self.costs = []
    
    def evaluate_performance(self) -> float:
        """ Compute the JNIS performance cost of the state estimator with the current set of noise parameters.
        """
        assert len(self.population) > 0, "Population performance must not be empty."
        assert len(self.average_costs) > 0, "Population performance must not be empty."
        assert self.total_time > 0.0, "Total time must be greater than 0."
        evaluation = np.log(np.sum(self.average_costs) / (self.total_time * self.dof))
        self.population[-1]['JNIS'] = evaluation

    def fit(self) -> None:
        """ Fit the surrogate model to the current population of noise parameters and JNIS performance costs.
        """
        assert len(self.population) > 0, "Population must not be empty."
        assert len(self.population[-1]['parameters']) == self.num_optimization_parameters, \
            "Number of noise parameters {} must match the number of optimization parameters which is {}" \
            .format(len(self.population[-1]['parameters']),self.num_optimization_parameters)
        assert self.population[-1]['JNIS'] is not None, "JNIS performance cost must be provided."
        # optimizer_idx = np.min([self.population[p]['JNIS'] for p in range(len(self.population))])
        self.surrogate_model = tfd.StudentTProcess(
            df=self.dof,
            kernel=self.kernel,
            index_points=self.index_points,
            mean_fn=self.mean_fn,
        )

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

    def populate(self, noise_parameters: list) -> None:
        """ Populate the surrogate model with a set of provided noise parameters
        with the corresponding list of JNIS performance costs.
        """
        set_noise_parameters = {
            'parameters': noise_parameters,
            'JNIS': None,
        }
        self.population.append(set_noise_parameters)

    def mean_fn(self, x: np.ndarray) -> np.ndarray:
        """ Define the mean function of the surrogate model.
        """
        return np.zeros_like(x)