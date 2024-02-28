import rclpy
from rclpy.node import Node

from march_shared_msgs.msg import IksFootPositions, StateEstimation

class MarchAieGaitPlanningNode(Node):

    def __init__(self):
        super().__init__('march_aie_gait_planning_node')

        self.state_estimation_subscription = self.create_subscription(
            StateEstimation,
            'state_estimation/state',
            self.state_estimation_callback,
            10
        )

        self.iks_foot_position_publisher = self.create_publisher(
            IksFootPositions,
            'ik_solver/buffer/input',
            10
        )

        self.get_logger().info('March AIE Gait Planning Node Started')

    def state_estimation_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg)

        iks_foot_positions_msg = IksFootPositions()
        iks_foot_positions_msg.header = msg.header
        iks_foot_positions_msg.left_foot_position.x = 0.0
        iks_foot_positions_msg.left_foot_position.y = 0.0
        iks_foot_positions_msg.left_foot_position.z = 0.0
        iks_foot_positions_msg.right_foot_position.x = 0.0
        iks_foot_positions_msg.right_foot_position.y = 0.0
        iks_foot_positions_msg.right_foot_position.z = 0.0

        self.iks_foot_position_publisher.publish(iks_foot_positions_msg)


def main(args=None):
    rclpy.init(args=args)
    march_aie_gait_planning_node = MarchAieGaitPlanningNode()
    rclpy.spin(march_aie_gait_planning_node)
    march_aie_gait_planning_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()