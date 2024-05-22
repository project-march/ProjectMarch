"""
Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy
"""

import pytest
import numpy as np
from march_sensor_fusion_optimization.bayesian_optimization import BayesianOptimizer

dof = 4.0
max_iter = 1000
min_obs = 1e-6
amplitude = 1.0
length_scale = 1.0
population_size = 10
param_filepath = 'test/mocks/dummy_parameters.yaml'

def test_bayesian_optimizer_can_initialize():
    optimizer = BayesianOptimizer(dof, max_iter, min_obs, param_filepath, amplitude, length_scale, population_size)
    assert optimizer is not None
    assert optimizer.bounds.shape == (5, 2)

def test_bayesian_optimizer_can_populate_initially():
    optimizer = BayesianOptimizer(dof, max_iter, min_obs, param_filepath, amplitude, length_scale, population_size)
    optimizer.populate(np.array([1.0, 2.0, 3.0, 4.0, 5.0]))
    assert len(optimizer.X_sample) == 1
    assert len(optimizer.y_sample) == 1
    assert optimizer.X_sample.shape == (1, 5)
    assert optimizer.y_sample.shape == (1, 1)

def test_bayesian_optimizer_can_populate_existing():
    optimizer = BayesianOptimizer(dof, max_iter, min_obs, param_filepath, amplitude, length_scale, population_size)
    optimizer.populate(np.array([1.0, 2.0, 3.0, 4.0, 5.0]))
    optimizer.populate(np.array([11.0, 12.0, 13.0, 14.0, 15.0]))
    assert len(optimizer.X_sample) == 2
    assert len(optimizer.y_sample) == 2
    assert optimizer.X_sample.shape == (2, 5)
    assert optimizer.y_sample.shape == (2, 1)

def test_bayesian_optimizer_can_configure_noise_parameters():
    optimizer = BayesianOptimizer(dof, max_iter, min_obs, param_filepath, amplitude, length_scale, population_size)
    optimizer.populate(np.array([1.0, 2.0, 3.0, 4.0, 5.0]))
    optimizer.configure_noise_parameters(optimizer.X_sample[0])
    assert optimizer.parameter_handler.get_parameter('process_noise', 'foot_position') == optimizer.X_sample[0][0]
    assert optimizer.parameter_handler.get_parameter('process_noise', 'foot_slippage') == optimizer.X_sample[0][1]
    assert optimizer.parameter_handler.get_parameter('observation_noise', 'foot_position') == optimizer.X_sample[0][2]
    assert optimizer.parameter_handler.get_parameter('observation_noise', 'foot_slippage') == optimizer.X_sample[0][3]
    assert optimizer.parameter_handler.get_parameter('observation_noise', 'joint_position') == optimizer.X_sample[0][4]
    assert optimizer.total_time == 0.0

def test_bayesian_optimizer_can_select_random_noise_parameters():
    optimizer = BayesianOptimizer(dof, max_iter, min_obs, param_filepath, amplitude, length_scale, population_size)
    optimizer.select_random_noise_parameters()
    assert len(optimizer.X_sample) == 1
    assert len(optimizer.X_sample[0]) == 5
    for i in range(5):
        assert  optimizer.X_sample[0][i] >= optimizer.bounds[i][0] and \
                optimizer.X_sample[0][0] <= optimizer.bounds[i][1]

def test_bayesian_optimizer_can_collect_performance():
    optimizer = BayesianOptimizer(dof, max_iter, min_obs, param_filepath, amplitude, length_scale, population_size)
    optimizer.select_random_noise_parameters()
    assert len(optimizer.X_sample) == 1
    for i in range(10):
        optimizer.collect_performance_cost(1e-6, 0.1)
        assert len(optimizer.costs) == i + 1
        assert optimizer.costs[i] == 1e-6
        assert pytest.approx(optimizer.total_time) == 0.1 * (i + 1)

def test_bayesian_optimizer_can_compute_average_performance():
    optimizer = BayesianOptimizer(dof, max_iter, min_obs, param_filepath, amplitude, length_scale, population_size)
    optimizer.select_random_noise_parameters()
    optimizer.costs = [1.0, 2.0, 3.0, 4.0]
    optimizer.compute_average_performance_cost()
    assert pytest.approx(optimizer.average_costs[0]) == 2.5
    assert len(optimizer.costs) == 0

def test_bayesian_optimizer_can_evaluate_performance():
    optimizer = BayesianOptimizer(dof, max_iter, min_obs, param_filepath, amplitude, length_scale, population_size)
    optimizer.select_random_noise_parameters()
    optimizer.total_time = 10.0
    optimizer.average_costs = [4.0, 8.0, 12.0, 16.0]
    optimizer.evaluate_performance_cost()
    assert len(optimizer.y_sample) == 1
    assert pytest.approx(optimizer.y_sample[-1, 0]) == 0.0

def test_bayesian_optimizer_can_fit():
    optimizer = BayesianOptimizer(dof, max_iter, min_obs, param_filepath, amplitude, length_scale, population_size)
    for _ in range(population_size):
        optimizer.select_random_noise_parameters()
    try:
        optimizer.fit()
        assert optimizer.surrogate_model is not None
        assert optimizer.mu.shape == (1, 5)
        assert optimizer.sigma.shape == (1, 5)
    except Exception as e:
        pytest.fail(f"BayesianOptimizer.fit() raised an exception: {e}")

def test_bayesian_optimizer_can_sample():
    optimizer = BayesianOptimizer(dof, max_iter, min_obs, param_filepath, amplitude, length_scale, population_size)
    for _ in range(population_size):
        optimizer.select_random_noise_parameters()
    optimizer.fit()
    assert optimizer.surrogate_model.sample(1).shape == (1, 5)