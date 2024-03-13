import rclpy
from rclpy.node import Node

from march_shared_msgs.msg import IksFootPositions, StateEstimation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import numpy as np

AIE_OSCILLATION_DEGREE = 5.0 * np.pi / 180.0

class KeyframeNode(Node):

    def __init__(self):
        super().__init__('keyframe_node')

        self.joint_trajectory_publisher = self.create_publisher(
            JointTrajectory,
            'joint_trajectory_controller/joint_trajectory',
            10
        )

        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        msg = JointTrajectory()
        msg.joint_names = ['left_hip_aa', 'left_hip_fe', 'left_knee', 'left_ankle', 'right_hip_aa', 'right_hip_fe', 'right_knee', 'right_ankle']
        msg.header.stamp = self.get_clock().now().to_msg()
        
        point = JointTrajectoryPoint()
        point.positions = [-0.11564, 0.0, 1.49467, 0.0, -0.0501945, 0.0, 0.0, -0.033172]
        point.velocities = [0.0] * 8
        point.accelerations = [0.0] * 8
        point.effort = [0.0] * 8
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(10e7)

        msg.points.append(point)
        self.joint_trajectory_publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    keyframe_node = KeyframeNode()
    rclpy.spin(keyframe_node)
    keyframe_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()