"""
Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy
"""

import yaml

class ParametersHandler:

    def __init__(self, path: str) -> None:
        self.path = path
        self.parameters = self.load_parameters()

    def load_parameters(self) -> dict:
        with open(self.path, 'r') as file:
            return yaml.safe_load(file)
        
    def get_parameter(self, type: str, parameter: str) -> dict:
        return self.parameters['state_estimator']['ros__parameters']['noise_parameters'][type][parameter]
    
    def set_parameter(self, type: str, parameter: str, values: list) -> None:
        self.parameters['state_estimator']['ros__parameters']['noise_parameters'][type][parameter] = values

    def save_parameters(self, path: str = None) -> None:
        if path is None:
            path = self.path
        with open(path, 'w') as file:
            yaml.dump(self.parameters, file)
