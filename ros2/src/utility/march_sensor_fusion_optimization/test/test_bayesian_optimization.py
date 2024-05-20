"""
Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy
"""

import pytest
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
    assert len(optimizer.initial_population) == population_size
    assert type(optimizer.initial_population) == list
    assert len(optimizer.initial_population[0]) == 5
    assert optimizer.index_points.shape == (5, population_size)

def test_bayesian_optimizer_can_configure_noise_parameters():
    optimizer = BayesianOptimizer(dof, max_iter, min_obs, param_filepath, amplitude, length_scale, population_size)
    optimizer.configure_noise_parameters(optimizer.initial_population[0])
    assert optimizer.parameter_handler.get_parameter('process_noise', 'foot_position') == optimizer.initial_population[0][0]
    assert optimizer.parameter_handler.get_parameter('process_noise', 'foot_slippage') == optimizer.initial_population[0][1]
    assert optimizer.parameter_handler.get_parameter('observation_noise', 'foot_position') == optimizer.initial_population[0][2]
    assert optimizer.parameter_handler.get_parameter('observation_noise', 'foot_slippage') == optimizer.initial_population[0][3]
    assert optimizer.parameter_handler.get_parameter('observation_noise', 'joint_position') == optimizer.initial_population[0][4]
    assert optimizer.total_time == 0.0

def test_bayesian_optimizer_can_select_random_noise_parameters():
    optimizer = BayesianOptimizer(dof, max_iter, min_obs, param_filepath, amplitude, length_scale, population_size)
    optimizer.select_random_noise_parameters()
    assert len(optimizer.population) == 1
    assert len(optimizer.population[0]['parameters']) == 5
    assert len(optimizer.initial_population) == population_size - 1

def test_bayesian_optimizer_can_collect_performance():
    optimizer = BayesianOptimizer(dof, max_iter, min_obs, param_filepath, amplitude, length_scale, population_size)
    optimizer.select_random_noise_parameters()
    for i in range(10):
        optimizer.collect_performance(1e-6, 0.1)
        assert len(optimizer.population) == 1
        assert len(optimizer.costs) == i + 1
        assert optimizer.costs[i] == 1e-6
        assert pytest.approx(optimizer.total_time) == 0.1 * (i + 1)

def test_bayesian_optimizer_can_compute_average_performance():
    optimizer = BayesianOptimizer(dof, max_iter, min_obs, param_filepath, amplitude, length_scale, population_size)
    optimizer.select_random_noise_parameters()
    optimizer.costs = [1.0, 2.0, 3.0, 4.0]
    optimizer.compute_average_performance()
    assert pytest.approx(optimizer.average_costs[0]) == 2.5
    assert len(optimizer.costs) == 0

def test_bayesian_optimizer_can_evaluate_performance():
    optimizer = BayesianOptimizer(dof, max_iter, min_obs, param_filepath, amplitude, length_scale, population_size)
    optimizer.select_random_noise_parameters()
    optimizer.total_time = 10.0
    optimizer.average_costs = [4.0, 8.0, 12.0, 16.0]
    optimizer.evaluate_performance()
    assert len(optimizer.population) == 1
    assert len(optimizer.population[0]['parameters']) == 5
    assert pytest.approx(optimizer.population[0]['JNIS']) == 0.0

def test_bayesian_optimizer_can_fit():
    optimizer = BayesianOptimizer(dof, max_iter, min_obs, param_filepath, amplitude, length_scale, population_size)
    optimizer.population = [
        {
            'parameters': [1.0, 2.0, 3.0, 4.0, 5.0],
            'JNIS': 0.0
        },
        {
            'parameters': [11.0, 12.0, 13.0, 14.0, 15.0],
            'JNIS': 1.0
        },
        {
            'parameters': [21.0, 22.0, 23.0, 24.0, 25.0],
            'JNIS': 2.0
        }
    ]
    try:
        optimizer.fit()
    except Exception as e:
        pytest.fail(f"BayesianOptimizer.fit() raised an exception: {e}")