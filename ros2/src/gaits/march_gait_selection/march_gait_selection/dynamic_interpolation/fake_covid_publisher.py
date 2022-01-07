import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point

import random

NODE_NAME = "fake_covid_publisher"


class FakeCovidPublisher(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        self.left_foot_publisher = self.create_publisher(
            Point,
            "/foot_position/left",
            10,
        )

        self.right_foot_publisher = self.create_publisher(
            Point,
            "/foot_position/right",
            10,
        )

        self.create_timer(0.1, self.publish_locations)

    def publish_locations(self):
        point = Point()
        point.x = random.uniform(0.2, 0.5)
        point.y = 0.0
        point.z = 0.0

        self.left_foot_publisher.publish(point)
        self.right_foot_publisher.publish(point)


def main():
    rclpy.init()
    fake_covid_publisher = FakeCovidPublisher()
    rclpy.spin(fake_covid_publisher)


if __name__ == "__main__":
    main()
