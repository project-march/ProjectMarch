"""
Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy
"""

from enum import Enum

class BayesianOptimizationStates(Enum):
    STATE_INITIALIZATION= 0
    STATE_RANDOMIZATION = 1
    STATE_CONFIGURATION = 2
    STATE_ACTIVATION = 3
    STATE_COLLECTION = 4
    STATE_DEACTIVATION = 5
    STATE_CLEANUP = 6
    STATE_FIT = 7
    STATE_ACQUISITION = 8
    STATE_OPTIMIZATION = 9
    STATE_DONE = 10

class BayesianOptimizationModes(Enum):
    MODE_FIT = 0
    MODE_OPTIMIZATION = 1

class StateHandler:

    def __init__(self, max_collection_period: float, max_fit_iterations: int, max_optimization_iterations: int, min_observation_change: float) -> None:
        self.max_collection_period = max_collection_period
        self.max_fit_iterations = max_fit_iterations
        self.max_optimization_iterations = max_optimization_iterations
        self.min_observation_change = min_observation_change

        self.current_state = BayesianOptimizationStates.STATE_INITIALIZATION
        self.current_collection_time = 0.0
        self.current_observation_change = float('inf')
        self.current_iterations = 0
        self.current_mode = None

    def transition_state(self) -> BayesianOptimizationStates:
        # State transition logic

        # Initialization
        if self.current_state == BayesianOptimizationStates.STATE_INITIALIZATION:
            self.current_mode = BayesianOptimizationModes.MODE_FIT
            self.current_state = BayesianOptimizationStates.STATE_RANDOMIZATION
            return BayesianOptimizationStates.STATE_RANDOMIZATION
        
        # Randomization
        elif self.current_state == BayesianOptimizationStates.STATE_RANDOMIZATION:
            self.current_state = BayesianOptimizationStates.STATE_CONFIGURATION
            return BayesianOptimizationStates.STATE_CONFIGURATION
        
        # Configuration
        elif self.current_state == BayesianOptimizationStates.STATE_CONFIGURATION:
            self.current_state = BayesianOptimizationStates.STATE_ACTIVATION
            return BayesianOptimizationStates.STATE_ACTIVATION
        
        # Activation
        elif self.current_state == BayesianOptimizationStates.STATE_ACTIVATION:
            self.current_collection_time = 0.0
            self.current_state = BayesianOptimizationStates.STATE_COLLECTION
            return BayesianOptimizationStates.STATE_COLLECTION
        
        # Collection
        elif self.current_state == BayesianOptimizationStates.STATE_COLLECTION:
            if self.current_collection_time < self.max_collection_period:
                return BayesianOptimizationStates.STATE_COLLECTION
            else:
                self.current_state = BayesianOptimizationStates.STATE_DEACTIVATION
                return BayesianOptimizationStates.STATE_DEACTIVATION
            
        # Deactivation
        elif self.current_state == BayesianOptimizationStates.STATE_DEACTIVATION:
            self.current_iterations += 1
            self.current_state = BayesianOptimizationStates.STATE_CLEANUP
            return BayesianOptimizationStates.STATE_CLEANUP
        
        # Cleanup
        elif self.current_state == BayesianOptimizationStates.STATE_CLEANUP:
            if self.current_mode == BayesianOptimizationModes.MODE_FIT:
                if self.current_iterations < self.max_fit_iterations:
                    self.current_state = BayesianOptimizationStates.STATE_RANDOMIZATION
                    return BayesianOptimizationStates.STATE_RANDOMIZATION
                else:
                    self.current_mode = BayesianOptimizationModes.MODE_OPTIMIZATION
                    self.current_state = BayesianOptimizationStates.STATE_FIT
                    return BayesianOptimizationStates.STATE_FIT
            else:
                self.current_state = BayesianOptimizationStates.STATE_OPTIMIZATION
                return BayesianOptimizationStates.STATE_OPTIMIZATION
            
        # Fit
        elif self.current_state == BayesianOptimizationStates.STATE_FIT:
            self.current_state = BayesianOptimizationStates.STATE_ACQUISITION
            return BayesianOptimizationStates.STATE_ACQUISITION
        
        # Acquisition
        elif self.current_state == BayesianOptimizationStates.STATE_ACQUISITION:
            self.current_state = BayesianOptimizationStates.STATE_CONFIGURATION
            return BayesianOptimizationStates.STATE_CONFIGURATION
        
        # Optimization
        elif self.current_state == BayesianOptimizationStates.STATE_OPTIMIZATION:
            if self.current_iterations >= self.max_optimization_iterations or self.current_observation_change < self.min_observation_change:
                self.current_state = BayesianOptimizationStates.STATE_DONE
                return BayesianOptimizationStates.STATE_DONE
            else:
                self.current_state = BayesianOptimizationStates.STATE_ACQUISITION
                return BayesianOptimizationStates.STATE_ACQUISITION
            
        # Done
        elif self.current_state == BayesianOptimizationStates.STATE_DONE:
            return BayesianOptimizationStates.STATE_DONE
        
        else:
            return self.current_state
            
    def get_current_state(self) -> BayesianOptimizationStates:
        return self.current_state
   
    def update_collection_time(self, dt: float):
        self.current_collection_time += dt

    def update_observation_change(self, change: float):
        self.current_observation_change = change