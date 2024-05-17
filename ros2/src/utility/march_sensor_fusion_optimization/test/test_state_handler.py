"""
Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy
"""

from march_sensor_fusion_optimization.state_handler import BayesianOptimizationStates, BayesianOptimizationTransitions, StateHandler

def test_state_handler_initial_state():
    sh = StateHandler(BayesianOptimizationStates.STATE_CONFIGURATION)
    assert sh.get_current_state() == BayesianOptimizationStates.STATE_CONFIGURATION

def test_state_handler_transition_configuration_initialization():
    sh = StateHandler(BayesianOptimizationStates.STATE_CONFIGURATION)
    assert sh.transition_state(BayesianOptimizationTransitions.TRANSITION_CONFIGURATION_INITIALIZATION) == True
    assert sh.get_current_state() == BayesianOptimizationStates.STATE_INITALIZATION
    
def test_state_handler_transition_initialization_optimization():
    sh = StateHandler(BayesianOptimizationStates.STATE_INITALIZATION)
    assert sh.transition_state(BayesianOptimizationTransitions.TRANSITION_INITIALIZATION_OPTIMATION) == True
    assert sh.get_current_state() == BayesianOptimizationStates.STATE_OPTIMIZATION

def test_state_handler_transition_optimization_completion():
    sh = StateHandler(BayesianOptimizationStates.STATE_OPTIMIZATION)
    assert sh.transition_state(BayesianOptimizationTransitions.TRANSITION_OPTIMIZATION_COMPLETION) == True
    assert sh.get_current_state() == BayesianOptimizationStates.STATE_COMPLETION

def test_state_handler_transition_invalid():
    sh = StateHandler(BayesianOptimizationStates.STATE_COMPLETION)
    assert sh.transition_state(BayesianOptimizationTransitions.TRANSITION_CONFIGURATION_INITIALIZATION) == False
    assert sh.get_current_state() == BayesianOptimizationStates.STATE_COMPLETION
    assert sh.transition_state(BayesianOptimizationTransitions.TRANSITION_INITIALIZATION_OPTIMATION) == False
    assert sh.get_current_state() == BayesianOptimizationStates.STATE_COMPLETION
    assert sh.transition_state(BayesianOptimizationTransitions.TRANSITION_OPTIMIZATION_COMPLETION) == False
    assert sh.get_current_state() == BayesianOptimizationStates.STATE_COMPLETION
    assert sh.transition_state(None) == False
    assert sh.get_current_state() == BayesianOptimizationStates.STATE_COMPLETION

def test_state_handler_get_available_transitions():
    sh = StateHandler(BayesianOptimizationStates.STATE_CONFIGURATION)
    assert sh.get_available_transitions() == BayesianOptimizationTransitions.TRANSITION_CONFIGURATION_INITIALIZATION
    sh = StateHandler(BayesianOptimizationStates.STATE_INITALIZATION)
    assert sh.get_available_transitions() == BayesianOptimizationTransitions.TRANSITION_INITIALIZATION_OPTIMATION
    sh = StateHandler(BayesianOptimizationStates.STATE_OPTIMIZATION)
    assert sh.get_available_transitions() == BayesianOptimizationTransitions.TRANSITION_OPTIMIZATION_COMPLETION