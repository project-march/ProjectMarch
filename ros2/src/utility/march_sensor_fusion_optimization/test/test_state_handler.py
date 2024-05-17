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