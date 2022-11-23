from mujoco_interfaces.msg import MujocoSetControl
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState


class Mujoco_writerNode(Node):

    def __init__(self):
        """This node is responsible for sending any commands from
        our ROS systems to the Mujoco Sim node. it sends periodic
        commands to update the low level controller
        NOTE: Right now, it sends only a period command update.
        Later, we will update this to be a publisher which
        sends any commands obtained from another topic,
        with which the ROS control systems can interact
        """
        super().__init__("mujoco_writer")
        self.publisher = self.create_publisher(JointTrajectoryControllerState, 'mujoco_input', 10)
        CONTROL_PUBLISH_RATE = 0.5
        self.subscription = self.create_subscription(
            JointTrajectoryControllerState,
            'joint_trajectory_controller/state',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        # self.timer = self.create_timer(CONTROL_PUBLISH_RATE, self.timer_callback)
    
    def listener_callback(self, msg):
        """NOTE: we will replace this timer callback with a subscriber
        callback once we fully intergrate this with other packages.
        """
        # msg = MujocoSetControl()
        # msg.stamp = self.get_clock().now().to_msg()
        #This data is a placeholder, just for testing purposes
        # msg.reference_control = [0.0,0.0,0.0,5.0,0.0]
        # msg.mode = 1
        # self.get_logger().info('Listener callback triggered!')
        # self.get_logger().info(str(msg) + "\n")
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Mujoco_writerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
