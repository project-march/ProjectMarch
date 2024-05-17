"""
Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy
"""

from march_sensor_fusion_optimization.parameters_handler import ParametersHandler

filepath = 'test/mocks/dummy_parameters.yaml'

def set_new_parameters_in_parameters_handler(ph: ParametersHandler) -> None:
    ph.set_parameter('process_noise', 'linear_acceleration', [1e-6, 1e-6, 1e-6])
    ph.set_parameter('process_noise', 'angular_velocity', [1e-6, 1e-6, 1e-6])
    ph.set_parameter('process_noise', 'foot_position', [1e-6, 1e-6, 1e-6])
    ph.set_parameter('process_noise', 'accelerometer_bias', [1e-6, 1e-6, 1e-6])
    ph.set_parameter('process_noise', 'gyroscope_bias', [1e-6, 1e-6, 1e-6])
    ph.set_parameter('process_noise', 'foot_slippage', [1e-6, 1e-6, 1e-6])
    ph.set_parameter('observation_noise', 'foot_position', [1e6, 1e6, 1e6])
    ph.set_parameter('observation_noise', 'foot_slippage', [1e6, 1e6, 1e6])
    ph.set_parameter('observation_noise', 'joint_position', [1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6])

def test_parameters_handler_load_parameters():
    ph = ParametersHandler(filepath)
    assert ph.load_parameters() == {'state_estimator': {
                                        'ros__parameters': {
                                            'noise_parameters': {
                                                'process_noise': {
                                                        'linear_acceleration': [1e-12, 1e-12, 1e-12],
                                                        'angular_velocity': [1e-12, 1e-12, 1e-12],
                                                        'foot_position': [1e-12, 1e-12, 1e-12],
                                                        'accelerometer_bias': [1e-12, 1e-12, 1e-12],
                                                        'gyroscope_bias': [1e-12, 1e-12, 1e-12],
                                                        'foot_slippage': [1e-12, 1e-12, 1e-12],
                                                },
                                                'observation_noise': {
                                                        'foot_position': [1e3, 1e3, 1e3],
                                                        'foot_slippage': [1e3, 1e3, 1e3],
                                                        'joint_position': [1e3, 1e3, 1e3, 1e3, 1e3, 1e3, 1e3, 1e3, 1e3, 1e3],
                                                },
                                            },
                                            'optimization_parameters': {
                                                'process_noise': ['foot_position', 'foot_slippage'],
                                                'observation_noise': ['foot_position', 'foot_slippage', 'joint_position'],
                                            }
                                        }
                                    }}

def test_parameters_handler_get_noise_type_names():
    ph = ParametersHandler(filepath)
    assert ph.noise_type_names == ['process_noise', 'observation_noise']

def test_parameters_handler_get_noise_parameter_names():
    ph = ParametersHandler(filepath)
    assert ph.get_noise_parameter_names('process_noise') == ['linear_acceleration', 'angular_velocity', 'foot_position', 'accelerometer_bias', 'gyroscope_bias', 'foot_slippage']
    assert ph.get_noise_parameter_names('observation_noise') == ['foot_position', 'foot_slippage', 'joint_position']

def test_parameters_handler_get_parameter():
    ph = ParametersHandler(filepath)
    assert ph.get_parameter('process_noise', 'linear_acceleration') == [1e-12, 1e-12, 1e-12]
    assert ph.get_parameter('process_noise', 'angular_velocity') == [1e-12, 1e-12, 1e-12]
    assert ph.get_parameter('process_noise', 'foot_position') == [1e-12, 1e-12, 1e-12]
    assert ph.get_parameter('process_noise', 'accelerometer_bias') == [1e-12, 1e-12, 1e-12]
    assert ph.get_parameter('process_noise', 'gyroscope_bias') == [1e-12, 1e-12, 1e-12]
    assert ph.get_parameter('process_noise', 'foot_slippage') == [1e-12, 1e-12, 1e-12]
    assert ph.get_parameter('observation_noise', 'foot_position') == [1e3, 1e3, 1e3]
    assert ph.get_parameter('observation_noise', 'foot_slippage') == [1e3, 1e3, 1e3]
    assert ph.get_parameter('observation_noise', 'joint_position') == [1e3, 1e3, 1e3, 1e3, 1e3, 1e3, 1e3, 1e3, 1e3, 1e3]

def test_parameters_handler_set_parameter():
    ph = ParametersHandler(filepath)
    set_new_parameters_in_parameters_handler(ph)
    assert ph.get_parameter('process_noise', 'linear_acceleration') == [1e-6, 1e-6, 1e-6]
    assert ph.get_parameter('process_noise', 'angular_velocity') == [1e-6, 1e-6, 1e-6]
    assert ph.get_parameter('process_noise', 'foot_position') == [1e-6, 1e-6, 1e-6]
    assert ph.get_parameter('process_noise', 'accelerometer_bias') == [1e-6, 1e-6, 1e-6]
    assert ph.get_parameter('process_noise', 'gyroscope_bias') == [1e-6, 1e-6, 1e-6]
    assert ph.get_parameter('process_noise', 'foot_slippage') == [1e-6, 1e-6, 1e-6]
    assert ph.get_parameter('observation_noise', 'foot_position') == [1e6, 1e6, 1e6]
    assert ph.get_parameter('observation_noise', 'foot_slippage') == [1e6, 1e6, 1e6]
    assert ph.get_parameter('observation_noise', 'joint_position') == [1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6]

def test_parameters_handler_save_parameters():
    ph = ParametersHandler(filepath)
    set_new_parameters_in_parameters_handler(ph)
    ph.save_parameters('test/mocks/result_test_parameters_handler_save_parameters.yaml')
    ph = ParametersHandler('test/mocks/result_test_parameters_handler_save_parameters.yaml')
    assert ph.load_parameters() == {'state_estimator': {
                                        'ros__parameters': {
                                            'noise_parameters': {
                                                'process_noise': {
                                                        'linear_acceleration': [1e-6, 1e-6, 1e-6],
                                                        'angular_velocity': [1e-6, 1e-6, 1e-6],
                                                        'foot_position': [1e-6, 1e-6, 1e-6],
                                                        'accelerometer_bias': [1e-6, 1e-6, 1e-6],
                                                        'gyroscope_bias': [1e-6, 1e-6, 1e-6],
                                                        'foot_slippage': [1e-6, 1e-6, 1e-6],
                                                },
                                                'observation_noise': {
                                                        'foot_position': [1e6, 1e6, 1e6],
                                                        'foot_slippage': [1e6, 1e6, 1e6],
                                                        'joint_position': [1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6]
                                                },
                                            },
                                            'optimization_parameters': {
                                                'process_noise': ['foot_position', 'foot_slippage'],
                                                'observation_noise': ['foot_position', 'foot_slippage', 'joint_position'],
                                            },
                                        },
                                    }}