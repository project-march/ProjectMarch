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