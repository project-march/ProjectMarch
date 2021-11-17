from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node
from dynamic_joint_trajectory_ros2 import DynamicJointTrajectory
from trajectory_msgs import msg as trajectory_msg


class PublishJointTrajectoryMsg(Node):
    def __init__(self):
        super().__init__("publish_joint_trajectory_msg")
        self.publisher = self.create_publisher(
            trajectory_msg.JointTrajectory, "/test/joint_trajectory_msg", 5
        )

        self.subsciber = self.create_subscription(
            JointState, "/test/joint_states", self.calculate_joint_trajectories, 5
        )

    def calculate_joint_trajectories(self, joint_state):
        dynamic_joint_trajectory = DynamicJointTrajectory()
        dynamic_joint_trajectory.set_middle_state()
        dynamic_joint_trajectory.get_desired_state()
        dynamic_joint_trajectory.get_current_state(joint_state)
        position, velocity = dynamic_joint_trajectory.interpolate_setpoints()

        joint_trajectory_msg = dynamic_joint_trajectory.to_joint_trajectory_msg()
        self.publisher.publish(joint_trajectory_msg)
        print("Publishing joint_trajectory_msg")


def main(args=None):
    rclpy.init(args=args)
    publish_joint_trajectory_msg = PublishJointTrajectoryMsg()
    rclpy.spin(publish_joint_trajectory_msg)

    publish_joint_trajectory_msg.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
