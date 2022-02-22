# Ignore for flake8 because it errors on the RNG
# flake8: noqa
import signal
import sys
import random
import rclpy

from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from contextlib import suppress
from march_shared_msgs.msg import FootPosition
from .fake_covid_publisher import FakeCovidPublisher

from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH

NODE_NAME = "gait_preprocessor_node"


class GaitPreprocessor(Node):
    """Node for processing the points coming from the vision package, before sending them to the gait generation. This node can also be used to simulate fake points."""
    def __init__(self):
        super().__init__(NODE_NAME, automatically_declare_parameters_from_overrides=True)

        self._simulate_points = (
            self.get_parameter("simulate_points").get_parameter_value().bool_value
        )
        
        if self._simulate_points:
            self._fake_point_publisher = FakeCovidPublisher(self)
            self.create_timer(0.1, self._fake_point_publisher._publish_locations)
        else:
            
            self.create_subscription(
                FootPosition,
                "/foot_position/left",
                self._callback_left,
                DEFAULT_HISTORY_DEPTH,
            )

            self.create_subscription(
                FootPosition,
                "/foot_position/right",
                self._callback_right,
                DEFAULT_HISTORY_DEPTH,
            )


    def _callback_left(self, foot_location: FootPosition) -> None:
        """Callback for new left point from covid"""
        pass

    
    def _callback_right(self, foot_location: FootPosition) -> None:
        """Callback for new right point from covid"""
        pass

    def _parameter_callback(self, parameters: list) -> SetParametersResult:
        """Callback for when ros parameters are updated"""
        if self._simulate_points:
            self._fake_point_publisher._parameter_callback()


def main():
    rclpy.init()
    gait_preprocessor = GaitPreprocessor()

    gait_preprocessor.add_on_set_parameters_callback(
        lambda params: gait_preprocessor._parameter_callback(params)
    )

    signal.signal(signal.SIGTERM, sys_exit)

    with suppress(KeyboardInterrupt):
        rclpy.spin(gait_preprocessor)

    rclpy.shutdown()


if __name__ == "__main__":
    main()

def sys_exit(*_):
    sys.exit(0)
