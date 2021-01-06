import rclpy
from rclpy.node import Node

NODE_NAME = 'robot_information_node'


def main():
    """Starts the robot information node."""
    rclpy.init()

    node = RobotInformation()

    rclpy.spin(node)


class RobotInformation(Node):
    def __init__(self):
        super().__init__(NODE_NAME,
                         automatically_declare_parameters_from_overrides=True,
                         namespace='march')
