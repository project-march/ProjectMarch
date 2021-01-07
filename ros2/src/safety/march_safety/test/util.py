import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue


def set_input_device_safety_parameters(node: Node, send_errors_interval: int,
                                       input_device_connection_timeout: int):
    # Set parameters of march safety node
    set_parameters_client = node.create_client(SetParameters,
                                               "/march/safety_node/set_parameters")
    set_parameters_client.wait_for_service()
    send_errors_interval_parameter = Parameter(name='send_errors_interval',
                                               value=ParameterValue(type=2,
                                                                    integer_value=send_errors_interval))
    input_device_connection_timeout_parameter = Parameter(
        name='input_device_connection_timeout',
        value=ParameterValue(type=2,
                             integer_value=input_device_connection_timeout))
    future = set_parameters_client.call_async(SetParameters.Request(
        parameters=[send_errors_interval_parameter,
                    input_device_connection_timeout_parameter]))
    rclpy.spin_until_future_complete(node, future)


class ErrorCounter:
    def __init__(self):
        self.count = 0

    def cb(self, _):
        self.count += 1