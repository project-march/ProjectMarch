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

from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH


class FakeCovidPublisher():
    """Class that spams fake possible foot locations, on topics '/foot_position/['left' or 'right']'.

    Can change distance locations during runtime, see '../launch/march_gait_preprocessor.launch.py'.
    """
    def __init__(self, node):

        self._node = node
        self._random_x = False
        self._random_y = False

        self._location_x = (
            self._node.get_parameter("location_x").get_parameter_value().double_value
        )
        self._location_y = (
            self._node.get_parameter("location_y").get_parameter_value().double_value
        )

        self._left_foot_publisher = self._node.create_publisher(
            msg_type=FootPosition,
            topic="/foot_position/left",
            qos_profile=DEFAULT_HISTORY_DEPTH,
        )

        self._right_foot_publisher = self._node.create_publisher(
            msg_type=FootPosition,
            topic="/foot_position/right",
            qos_profile=DEFAULT_HISTORY_DEPTH,
        )


    def _publish_locations(self) -> None:
        """Publishes the fake foot locations"""
        point_msg = FootPosition()
        point_msg.header.stamp = self._node.get_clock().now().to_msg()

        if self._random_x:
            point_msg.point.x = random.uniform(0.2, 0.5)
        else:
            point_msg.point.x = self._location_x

        if self._random_y:
            point_msg.point.y = random.uniform(0.0, 0.2)
        else:
            point_msg.point.y = self._location_y

        point_msg.point.z = 0.0

        self._left_foot_publisher.publish(point_msg)
        self._right_foot_publisher.publish(point_msg)


    def _parameter_callback(parameters: list) -> SetParametersResult:
        """Update parameter of fake_covid_publisher and return if
        this is done succesfully.

        :param parameters: list containing the changed parameters
        :type parameters: list

        :returns: whether or not the parameters were set succesfully
        :rtype: SetParametersResult
        """
        for param in parameters:
            if param.name == "location_x":
                if param.value == "random":
                    self._random_x = True
                    self._node.get_logger().info("x set to random.")
                else:
                    self._random_x = False
                    self._location_x = param.value
                    self._node.get_logger().info(f"x set to {param.value}")
            elif param.name == "location_y":
                if param.value == "random":
                    self._random_y = True
                    self._node.get_logger().info("y set to random.")
                else:
                    self._random_y = False
                    self._location_y = param.value
                    self._node.get_logger().info(f"y set to {param.value}")

        return SetParametersResult(successful=True)
