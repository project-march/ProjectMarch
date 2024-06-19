"""
Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy
"""

import yaml

class ParametersHandler:

    def __init__(self, path: str) -> None:
        self.path = path
        self.parameters = self.load_parameters()
        self.noise_type_names = [key for key in self.parameters['state_estimator']['ros__parameters']['noise_parameters'].keys()]

    def load_parameters(self) -> dict:
        with open(self.path, 'r') as file:
            return yaml.safe_load(file)
        
    def get_noise_parameter_names(self, type: str) -> list:
        return [key for key in self.parameters['state_estimator']['ros__parameters']['noise_parameters'][type].keys()]
        
    def get_parameter(self, type: str, parameter: str) -> dict:
        return self.parameters['state_estimator']['ros__parameters']['noise_parameters'][type][parameter]
    
    def set_parameter(self, type: str, parameter: str, values: list) -> None:
        self.parameters['state_estimator']['ros__parameters']['noise_parameters'][type][parameter] = values

    def get_optimization_parameter_names(self, type: str) -> list:
        return self.parameters['state_estimator']['ros__parameters']['optimization_parameters'][type]
    
    def get_num_optimization_parameters(self) -> int:
        size = 0
        for type in self.noise_type_names:
            size += len(self.get_optimization_parameter_names(type))
        return size
    
    def set_optimization_parameters(self, values: list) -> None:
        # new_process_params = values[0:len(self.get_optimization_parameter_names('process_noise'))]
        # for i, param in enumerate(self.get_optimization_parameter_names('process_noise')):
        #     self.parameters['state_estimator']['ros__parameters']['noise_parameters']['process_noise'][param] = new_process_params[i]
        # new_observation_params = values[len(new_process_params):]
        for i, param in enumerate(self.get_optimization_parameter_names('observation_noise')):
            # self.parameters['state_estimator']['ros__parameters']['noise_parameters']['observation_noise'][param] = new_observation_params[i]
            self.parameters['state_estimator']['ros__parameters']['noise_parameters']['observation_noise'][param] = values[i]

    def save_parameters(self, path: str = None) -> None:
        if path is None:
            path = self.path
        with open(path, 'w') as file:
            yaml.dump(self.parameters, file)
