from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node


class PublishFakeCurrentJointState(Node):
    def __init__(self):
        super().__init__("publish_fake_current_joint_state")
        self.publisher = self.create_publisher(JointState, "/test/joint_states", 5)
        self.timer = self.create_timer(5, self.timer_callback)

    def timer_callback(self):
        current_position = [0.0, 0.14, 0.31, 0.03, 0.03, -0.17, 0.14, 0.0]
        current_velocity = [0.0, 0.0, -0.35, 0.0, 0.0, 0.0, 0.0, 0.0]

        joint_state_msg = JointState()
        joint_state_msg.name = [
            "left_ankle",
            "left_knee",
            "left_hip_fe",
            "left_hip_aa",
            "right_hip_aa",
            "right_hip_fe",
            "right_knee",
            "right_ankle",
        ]

        joint_state_msg.position = current_position
        joint_state_msg.velocity = current_velocity

        self.publisher.publish(joint_state_msg)


def main(args=None):
    rclpy.init(args=args)
    publish_fake_current_joint_state = PublishFakeCurrentJointState()
    rclpy.spin(publish_fake_current_joint_state)

    publish_fake_current_joint_state.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
