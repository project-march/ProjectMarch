"""
Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy
"""

import pytest
from march_sensor_fusion_optimization.state_handler import StateHandler, BayesianOptimizationStates, BayesianOptimizationModes

collection_period = 0.05
max_collection_period = 1.0
max_fit_iterations = 100
max_optimization_iterations = 100
min_observation_change = 1e-6

def test_state_handler_initial_state():
    sh = StateHandler(max_collection_period, max_fit_iterations, max_optimization_iterations, min_observation_change)
    assert sh.current_state == BayesianOptimizationStates.STATE_INITIALIZATION

def test_state_handler_transition_state_initialization_to_randomization():
    sh = StateHandler(max_collection_period, max_fit_iterations, max_optimization_iterations, min_observation_change)
    sh.current_state = BayesianOptimizationStates.STATE_INITIALIZATION
    assert sh.transition_state() == BayesianOptimizationStates.STATE_RANDOMIZATION
    assert sh.current_state == BayesianOptimizationStates.STATE_RANDOMIZATION

def test_state_handler_transition_state_randomization_to_configuration():
    sh = StateHandler(max_collection_period, max_fit_iterations, max_optimization_iterations, min_observation_change)
    sh.current_state = BayesianOptimizationStates.STATE_RANDOMIZATION
    assert sh.transition_state() == BayesianOptimizationStates.STATE_CONFIGURATION
    assert sh.current_state == BayesianOptimizationStates.STATE_CONFIGURATION
    assert sh.current_collection_time == 0.0

def test_state_handler_transition_state_configuration_to_activation():
    sh = StateHandler(max_collection_period, max_fit_iterations, max_optimization_iterations, min_observation_change)
    sh.current_state = BayesianOptimizationStates.STATE_CONFIGURATION
    assert sh.transition_state() == BayesianOptimizationStates.STATE_ACTIVATION
    assert sh.current_state == BayesianOptimizationStates.STATE_ACTIVATION

def test_state_handler_transition_state_activation_to_collection():
    sh = StateHandler(max_collection_period, max_fit_iterations, max_optimization_iterations, min_observation_change)
    sh.current_state = BayesianOptimizationStates.STATE_ACTIVATION
    assert sh.transition_state() == BayesianOptimizationStates.STATE_COLLECTION
    assert sh.current_state == BayesianOptimizationStates.STATE_COLLECTION

def test_state_handler_transition_state_collection_to_collection():
    sh = StateHandler(max_collection_period, max_fit_iterations, max_optimization_iterations, min_observation_change)
    sh.current_state = BayesianOptimizationStates.STATE_COLLECTION
    assert sh.transition_state() == BayesianOptimizationStates.STATE_COLLECTION
    assert sh.current_state == BayesianOptimizationStates.STATE_COLLECTION

def test_state_handler_transition_state_collection_to_deactivation():
    sh = StateHandler(max_collection_period, max_fit_iterations, max_optimization_iterations, min_observation_change)
    sh.current_state = BayesianOptimizationStates.STATE_COLLECTION
    assert sh.current_state == BayesianOptimizationStates.STATE_COLLECTION
    assert sh.current_collection_time == 0.0
    num_collection_iterations = 0
    while (sh.current_collection_time < max_collection_period):
        sh.update_collection_time(collection_period)
        _  = sh.transition_state()
        num_collection_iterations += 1
    assert num_collection_iterations == 20
    assert sh.current_collection_time == pytest.approx(max_collection_period)
    assert sh.current_state == BayesianOptimizationStates.STATE_DEACTIVATION

def test_state_handler_transition_state_deactivation_to_cleanup():
    sh = StateHandler(max_collection_period, max_fit_iterations, max_optimization_iterations, min_observation_change)
    sh.current_state = BayesianOptimizationStates.STATE_DEACTIVATION
    assert sh.transition_state() == BayesianOptimizationStates.STATE_CLEANUP
    assert sh.current_state == BayesianOptimizationStates.STATE_CLEANUP
    assert sh.current_iterations == 1

def test_state_handler_transition_state_cleanup_to_randomization():
    sh = StateHandler(max_collection_period, max_fit_iterations, max_optimization_iterations, min_observation_change)
    sh.current_state = BayesianOptimizationStates.STATE_CLEANUP
    sh.current_mode = BayesianOptimizationModes.MODE_FIT
    assert sh.transition_state() == BayesianOptimizationStates.STATE_RANDOMIZATION
    assert sh.current_state == BayesianOptimizationStates.STATE_RANDOMIZATION

def test_state_handler_transition_state_cleanup_to_fit():
    sh = StateHandler(max_collection_period, max_fit_iterations, max_optimization_iterations, min_observation_change)
    sh.transition_state() # From initial state to randomization
    assert sh.current_state == BayesianOptimizationStates.STATE_RANDOMIZATION
    for i in range(max_fit_iterations):
        sh.transition_state() # From randomization to configuration
        assert sh.current_state == BayesianOptimizationStates.STATE_CONFIGURATION
        sh.transition_state() # From configuration to activation
        assert sh.current_state == BayesianOptimizationStates.STATE_ACTIVATION
        sh.transition_state() # From activation to collection
        assert sh.current_state == BayesianOptimizationStates.STATE_COLLECTION
        while (sh.current_collection_time < max_collection_period):
            sh.update_collection_time(collection_period)
            _ = sh.transition_state() # From collection to collection / deactivation
        assert sh.current_state == BayesianOptimizationStates.STATE_DEACTIVATION
        sh.transition_state() # From deactivation to cleanup
        assert sh.current_state == BayesianOptimizationStates.STATE_CLEANUP
        sh.transition_state() # From cleanup to randomization
        if i < max_fit_iterations - 1:
            assert sh.current_state == BayesianOptimizationStates.STATE_RANDOMIZATION
    assert sh.current_iterations == max_fit_iterations
    assert sh.current_state == BayesianOptimizationStates.STATE_FIT

def test_state_handler_transition_state_fit_to_acquisition():
    sh = StateHandler(max_collection_period, max_fit_iterations, max_optimization_iterations, min_observation_change)
    sh.current_state = BayesianOptimizationStates.STATE_FIT
    assert sh.transition_state() == BayesianOptimizationStates.STATE_ACQUISITION

def test_state_handler_transition_state_acquisition_to_configuration():
    sh = StateHandler(max_collection_period, max_fit_iterations, max_optimization_iterations, min_observation_change)
    sh.current_state = BayesianOptimizationStates.STATE_ACQUISITION
    assert sh.transition_state() == BayesianOptimizationStates.STATE_CONFIGURATION

def test_state_handler_transition_state_cleanup_to_optimization():
    sh = StateHandler(max_collection_period, max_fit_iterations, max_optimization_iterations, min_observation_change)
    sh.current_state = BayesianOptimizationStates.STATE_CLEANUP
    sh.current_mode = BayesianOptimizationModes.MODE_OPTIMIZATION
    assert sh.transition_state() == BayesianOptimizationStates.STATE_OPTIMIZATION

def test_state_handler_transition_state_optimization_to_acquisition():
    sh = StateHandler(max_collection_period, max_fit_iterations, max_optimization_iterations, min_observation_change)
    sh.current_state = BayesianOptimizationStates.STATE_OPTIMIZATION
    assert sh.transition_state() == BayesianOptimizationStates.STATE_ACQUISITION

def test_state_handler_transition_state_optimization_to_done():
    sh = StateHandler(max_collection_period, max_fit_iterations, max_optimization_iterations, min_observation_change)
    sh.current_state = BayesianOptimizationStates.STATE_OPTIMIZATION
    sh.current_iterations = max_optimization_iterations
    sh.current_observation_change = min_observation_change
    assert sh.transition_state() == BayesianOptimizationStates.STATE_DONE

def test_state_handler_complete_process_until_max_optimization_iterations_reached():
    sh = StateHandler(max_collection_period, max_fit_iterations, max_optimization_iterations, min_observation_change)
    assert sh.current_state == BayesianOptimizationStates.STATE_INITIALIZATION
    assert sh.transition_state() == BayesianOptimizationStates.STATE_RANDOMIZATION
    for i in range(max_fit_iterations):
        assert sh.transition_state() == BayesianOptimizationStates.STATE_CONFIGURATION
        assert sh.transition_state() == BayesianOptimizationStates.STATE_ACTIVATION
        assert sh.transition_state() == BayesianOptimizationStates.STATE_COLLECTION
        while (sh.current_collection_time < max_collection_period):
            sh.update_collection_time(collection_period)
            _ = sh.transition_state()
        assert sh.current_state == BayesianOptimizationStates.STATE_DEACTIVATION
        assert sh.transition_state() == BayesianOptimizationStates.STATE_CLEANUP
        if i < max_fit_iterations - 1:
            assert sh.transition_state() == BayesianOptimizationStates.STATE_RANDOMIZATION
        else:
            assert sh.transition_state() == BayesianOptimizationStates.STATE_FIT
            assert sh.transition_state() == BayesianOptimizationStates.STATE_ACQUISITION
    for i in range(max_optimization_iterations):
        assert sh.transition_state() == BayesianOptimizationStates.STATE_CONFIGURATION
        assert sh.transition_state() == BayesianOptimizationStates.STATE_ACTIVATION
        assert sh.transition_state() == BayesianOptimizationStates.STATE_COLLECTION
        while (sh.current_collection_time < max_collection_period):
            sh.update_collection_time(collection_period)
            _ = sh.transition_state()
        assert sh.current_state == BayesianOptimizationStates.STATE_DEACTIVATION
        assert sh.transition_state() == BayesianOptimizationStates.STATE_CLEANUP
        assert sh.transition_state() == BayesianOptimizationStates.STATE_OPTIMIZATION
        if i < max_optimization_iterations - 1:
            assert sh.transition_state() == BayesianOptimizationStates.STATE_ACQUISITION
    assert sh.transition_state() == BayesianOptimizationStates.STATE_DONE

def test_state_handler_complete_process_until_max_optimization_iterations_reached():
    observation_change = 1e5
    sh = StateHandler(max_collection_period, max_fit_iterations, max_optimization_iterations, min_observation_change)
    assert sh.current_state == BayesianOptimizationStates.STATE_INITIALIZATION
    assert sh.transition_state() == BayesianOptimizationStates.STATE_RANDOMIZATION
    for i in range(max_fit_iterations):
        assert sh.transition_state() == BayesianOptimizationStates.STATE_CONFIGURATION
        assert sh.transition_state() == BayesianOptimizationStates.STATE_ACTIVATION
        assert sh.transition_state() == BayesianOptimizationStates.STATE_COLLECTION
        while (sh.current_collection_time < max_collection_period):
            sh.update_collection_time(collection_period)
            _ = sh.transition_state()
        assert sh.current_state == BayesianOptimizationStates.STATE_DEACTIVATION
        assert sh.transition_state() == BayesianOptimizationStates.STATE_CLEANUP
        if i < max_fit_iterations - 1:
            assert sh.transition_state() == BayesianOptimizationStates.STATE_RANDOMIZATION
        else:
            assert sh.transition_state() == BayesianOptimizationStates.STATE_FIT
            assert sh.transition_state() == BayesianOptimizationStates.STATE_ACQUISITION
    while (sh.current_state != BayesianOptimizationStates.STATE_DONE):
        assert sh.transition_state() == BayesianOptimizationStates.STATE_CONFIGURATION
        assert sh.transition_state() == BayesianOptimizationStates.STATE_ACTIVATION
        assert sh.transition_state() == BayesianOptimizationStates.STATE_COLLECTION
        while (sh.current_collection_time < max_collection_period):
            sh.update_collection_time(collection_period)
            _ = sh.transition_state()
        assert sh.current_state == BayesianOptimizationStates.STATE_DEACTIVATION
        assert sh.transition_state() == BayesianOptimizationStates.STATE_CLEANUP
        assert sh.transition_state() == BayesianOptimizationStates.STATE_OPTIMIZATION
        _ = sh.transition_state()
        # Update the observation change
        sh.update_observation_change(observation_change)
        observation_change /= 10
    assert sh.transition_state() == BayesianOptimizationStates.STATE_DONE
