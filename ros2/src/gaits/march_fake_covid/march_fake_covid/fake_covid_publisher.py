# Ignore for flake8 because it errors on the RNG
# flake8: noqa
import signal
import sys

import random
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

import signal
import sys
from contextlib import suppress

from geometry_msgs.msg import Point
from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH

NODE_NAME = "fake_covid_publisher"


class FakeCovidPublisher(Node):
    """Class to spawn the node 'fake_covid_publisher' that will spam fake possible foot location,
       on topic '/foot_position/['left' or 'right']'.

    Can change distance locations during runtime, see '../launch/march_fake_covid.launch.py'.
    """
    def __init__(self):
        super().__init__(NODE_NAME, automatically_declare_parameters_from_overrides=True)
        self.random_x = False
        self.random_y = False

        self.location_x = (
            self.get_parameter("location_x").get_parameter_value().double_value
        )
        self.location_y = (
            self.get_parameter("location_y").get_parameter_value().double_value
        )

        self.left_foot_publisher = self.create_publisher(
            msg_type=Point,
            topic="/foot_position/left",
            qos_profile=DEFAULT_HISTORY_DEPTH,
        )

        self.right_foot_publisher = self.create_publisher(
            msg_type=Point,
            topic="/foot_position/right",
            qos_profile=DEFAULT_HISTORY_DEPTH,
        )

        self.create_timer(0.1, self.publish_locations)

    def publish_locations(self) -> None:
        """Publishes the fake foot locations"""
        point = Point()

        if self.random_x:
            point.x = random.uniform(0.2, 0.5)
        else:
            point.x = self.location_x

        if self.random_y:
            point.y = random.uniform(0.0, 0.2)
        else:
            point.y = self.location_y

        point.z = 0.0

        self.left_foot_publisher.publish(point)
        self.right_foot_publisher.publish(point)


def sys_exit(*_):
    sys.exit(0)


def main():
    rclpy.init()
    fake_covid_publisher = FakeCovidPublisher()

    fake_covid_publisher.add_on_set_parameters_callback(
        lambda params: parameter_callback(fake_covid_publisher, params)
    )

    signal.signal(signal.SIGTERM, sys_exit)

    with suppress(KeyboardInterrupt):
        rclpy.spin(fake_covid_publisher)

    rclpy.shutdown()


def parameter_callback(
    fake_covid_publisher: FakeCovidPublisher, parameters: list
) -> SetParametersResult:
    """Update parameter of fake_covid_publisher and return if
    this is done succesfully.

    :param fake_covid_publisher: instance of the fake_covid_publisher class
    :type fake_covid_publisher: FakeCovidPublisher
    :param parameters: list containing the changed parameters
    :type parameters: list

    :returns: whether or not the parameters were set succesfully
    :rtype: SetParametersResult
    """
    for param in parameters:
        if param.name == "location_x":
            if param.value == "random":
                fake_covid_publisher.random_x = True
                fake_covid_publisher.get_logger().info("x set to random.")
            else:
                fake_covid_publisher.random_x = False
                fake_covid_publisher.location_x = param.value
                fake_covid_publisher.get_logger().info(f"x set to {param.value}")
        elif param.name == "location_y":
            if param.value == "random":
                fake_covid_publisher.random_y = True
                fake_covid_publisher.get_logger().info("y set to random.")
            else:
                fake_covid_publisher.random_y = False
                fake_covid_publisher.location_y = param.value
                fake_covid_publisher.get_logger().info(f"y set to {param.value}")

    return SetParametersResult(successful=True)


if __name__ == "__main__":
    main()
