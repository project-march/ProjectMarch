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

        self.current_stance_leg = 0b11
        self.desired_stance_leg = 0b11
        self.step_height = 0.05
        self.step_width = 0.2
        
        self.home_stance = self.generate_home_stance()

        self.get_logger().info('March AIE Gait Planning Node Started')

    def state_estimation_callback(self, msg):
        self.current_stance_leg = msg.stance_leg
        self.publish_iks_foot_positions(self.home_stance)

    def publish_iks_foot_positions(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.iks_foot_position_publisher.publish(msg)

    def generate_home_stance(self):
        home_stance = IksFootPositions()
        home_stance.left_foot_position.x = 0.3
        home_stance.left_foot_position.y = 0.12
        home_stance.left_foot_position.z = -0.7
        home_stance.right_foot_position.x = 0.3
        home_stance.right_foot_position.y = -0.12
        home_stance.right_foot_position.z = -0.7
        return home_stance

def main(args=None):
    rclpy.init(args=args)
    march_aie_gait_planning_node = MarchAieGaitPlanningNode()
    rclpy.spin(march_aie_gait_planning_node)
    march_aie_gait_planning_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()