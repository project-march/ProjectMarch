"""
Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy
"""

import rclpy
from rclpy.node import Node

from march_shared_msgs.msg import StateEstimation

from march_sensor_fusion_optimization.bayesian_optimizer import BayesianOptimizer

class SensorFusionOptimizerNode(Node):

    def __init__(self) -> None:
        super().__init__('sensor_fusion_optimizer')

        self.performance_cost = 1e15
        self.bayesian_optimizer = BayesianOptimizer()

        self.state_estimation_subscriber = self.create_subscription(StateEstimation, 'state_estimation/state', self.state_estimation_callback, 10)

        self.get_logger().info('Sensor Fusion Optimizer Node has been initialized.')

    def state_estimation_callback(self, msg: StateEstimation) -> None:
        self.performance_cost = msg.performance_cost


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionOptimizerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
