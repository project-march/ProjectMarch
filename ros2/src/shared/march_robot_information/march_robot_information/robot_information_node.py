"""Author: Bas Volkers, MVI."""
from typing import List, Optional

import rclpy
from march_shared_msgs.srv import GetJointNames
from march_utility.utilities.node_utils import get_joint_names_from_robot, on_urdf_online
from rclpy.node import Node
from rclpy.parameter import Parameter
from contextlib import suppress

NODE_NAME = "robot_information_node"


def main():
    """Starts the robot information node."""
    rclpy.init()

    node = RobotInformation()

    with suppress(KeyboardInterrupt):
        rclpy.spin(node)

    rclpy.shutdown()


class RobotInformation(Node):
    """RobotInformation is a simple node that holds additional information about the march robot in its parameters."""

    def __init__(self, joint_names: Optional[List[str]] = None):
        super().__init__(NODE_NAME)
        self.get_parameter_clients = {}
        self.declare_parameter("joint_names")

        # If the given joint names are not None, use that value for the node
        # Otherwise get the value in the parameters of the node
        # If that value is an empty list, query the joint_names from the
        # robot_state_publisher node
        if joint_names is not None:
            self.joint_names = joint_names
            self.set_ready()
        else:
            joint_names = self.get_parameter("joint_names").get_parameter_value().string_array_value
            if len(joint_names) > 0:
                self.joint_names = sorted(joint_names)
                self.set_ready()
            else:

                def set_joint_names(robot):
                    self.joint_names = get_joint_names_from_robot(robot)
                    self.set_ready()

                on_urdf_online(self, set_joint_names)

    def set_ready(self):
        """Makes the services available."""
        self.set_parameters([Parameter(name="joint_names", value=self.joint_names)])
        self.create_service(GetJointNames, "robot_information/get_joint_names", self.get_joint_names_cb)

    def get_joint_names_cb(self, _, response):
        """Return the joint names."""
        response.joint_names = self.joint_names
        return response
