import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from march_shared_msgs.msg import ExoMode, IksFootPositions

class MPCPlanningNode(Node):

    def __init__(self):
        super().__init__('march_mpc_planning')

        self.declare_parameter('exo_mode_topic', '/exo_mode')

        self.clock_sub = self.create_subscription(Header, 'state_estimation/clock', self.clock_callback, 1)
        self.exo_mode_sub = self.create_subscription(ExoMode, 'current_mode', self.exo_mode_callback, 1)
        self.iks_foot_positions_pub = self.create_publisher(IksFootPositions, 'ik_solver/buffer/input', 10)

        self.current_mode = -1

    def exo_mode_callback(self, msg):
        self.get_logger().info('Received exo mode: {}'.format(msg.mode))
        self.current_mode = msg.mode


    def clock_callback(self, msg):
        if self.current_mode not in [ExoMode.BALANCESTAND]:
            return
        
        foot_positions = IksFootPositions()
        foot_positions.header = msg
        foot_positions.left_foot_position.x = 0.19
        foot_positions.left_foot_position.y = 0.22
        foot_positions.left_foot_position.z = -0.9
        foot_positions.right_foot_position.x = 0.19
        foot_positions.right_foot_position.y = 0.22
        foot_positions.right_foot_position.z = -0.9
        self.iks_foot_positions_pub.publish(foot_positions)

def main():
    rclpy.init()
    node = MPCPlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
