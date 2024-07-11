import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from march_shared_msgs.msg import ExoMode, IksFootPositions

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class MPCPlanningNode(Node):

    def __init__(self):
        super().__init__('march_mpc_planning')

        self.declare_parameter('exo_mode_topic', '/exo_mode')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.clock_sub = self.create_subscription(Header, 'state_estimation/clock', self.clock_callback, 1)
        self.exo_mode_sub = self.create_subscription(ExoMode, 'current_mode', self.exo_mode_callback, 1)
        self.iks_foot_positions_pub = self.create_publisher(IksFootPositions, 'ik_solver/buffer/input', 10)

        self.left_foot_position_pub = self.create_publisher(PointStamped, 'ik_solver/desired/foot_position/left', 10)
        self.right_foot_position_pub = self.create_publisher(PointStamped, 'ik_solver/desired/foot_position/right', 10)

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
        foot_positions.left_foot_position.z = -0.92
        foot_positions.right_foot_position.x = 0.19
        foot_positions.right_foot_position.y = -0.22
        foot_positions.right_foot_position.z = -0.92
        self.iks_foot_positions_pub.publish(foot_positions)

        left_foot_position = PointStamped()
        left_foot_position.header.stamp = msg.stamp
        left_foot_position.header.frame_id = 'backpack'
        left_foot_position.point = foot_positions.left_foot_position
        self.left_foot_position_pub.publish(left_foot_position)

        right_foot_position = PointStamped()
        right_foot_position.header.stamp = msg.stamp
        right_foot_position.header.frame_id = 'backpack'
        right_foot_position.point = foot_positions.right_foot_position
        self.right_foot_position_pub.publish(right_foot_position)


def main():
    rclpy.init()
    node = MPCPlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
