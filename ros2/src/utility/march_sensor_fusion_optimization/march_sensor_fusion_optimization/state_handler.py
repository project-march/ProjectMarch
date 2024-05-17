"""
Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy
"""

from enum import Enum

class BayesianOptimizationStates(Enum):
    STATE_CONFIGURATION = 0
    STATE_INITALIZATION = 1
    STATE_OPTIMIZATION = 2
    STATE_COMPLETION = 3

class BayesianOptimizationTransitions(Enum):
    TRANSITION_CONFIGURATION_INITIALIZATION = 0
    TRANSITION_INITIALIZATION_OPTIMATION = 1
    TRANSITION_OPTIMIZATION_COMPLETION = 2

class StateHandler:

    def __init__(self, initial_state: BayesianOptimizationStates) -> None:
        self.current_state = initial_state

    def transition_state(self, transition: BayesianOptimizationTransitions) -> bool:
        if self.current_state == BayesianOptimizationStates.STATE_CONFIGURATION:
            if transition == BayesianOptimizationTransitions.TRANSITION_CONFIGURATION_INITIALIZATION:
                self.current_state = BayesianOptimizationStates.STATE_INITALIZATION
                return True
        elif self.current_state == BayesianOptimizationStates.STATE_INITALIZATION:
            if transition == BayesianOptimizationTransitions.TRANSITION_INITIALIZATION_OPTIMATION:
                self.current_state = BayesianOptimizationStates.STATE_OPTIMIZATION
                return True
        elif self.current_state == BayesianOptimizationStates.STATE_OPTIMIZATION:
            if transition == BayesianOptimizationTransitions.TRANSITION_OPTIMIZATION_COMPLETION:
                self.current_state = BayesianOptimizationStates.STATE_COMPLETION
                return True
        else:
            return False
    
    def get_current_state(self) -> BayesianOptimizationStates:
        return self.current_state