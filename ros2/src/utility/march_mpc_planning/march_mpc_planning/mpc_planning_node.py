"""
Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy
"""

import rclpy
from rclpy.node import Node

import numpy as np

from march_shared_msgs.msg import IksFootPositions
from march_shared_msgs.msg import StateEstimation
from march_shared_msgs.msg import ExoMode
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray, PoseStamped

class MpcPlanningNode(Node):

    def __init__(self):
        super().__init__('mpc_planning_node')

        self.current_mode = -1
        self.current_left_foot_position = None
        self.current_right_foot_position = None
        self.dt = None
        self.leg_length = 0.89

        self.homestance_left_foot_position = [0.19, 0.22, -0.89]
        self.homestance_right_foot_position = [0.19, -0.22, -0.89]
        self.bezier_trajectory = []
        self.current_swing_leg = None

        self.exo_mode_subscriber = self.create_subscription(
            ExoMode, 'current_mode', self.exo_mode_callback, 10)
        self.state_estimation_subscriber = self.create_subscription(
            StateEstimation, 'state_estimation/state', self.state_estimation_callback, 10)
        self.iks_foot_positions_publisher = self.create_publisher(
            IksFootPositions, 'ik_solver/buffer/input', 10)
        self.desired_foot_positions_subscriber = self.create_subscription(
            PoseArray, 'mpc_solver/buffer/output', self.desired_foot_positions_callback, 10)
        self.bezier_trajectory_publisher = self.create_publisher(
            Path, 'mpc_solver/swing_trajectory', 10)

        self.get_logger().info('MPC Planning Node is running.')

    def exo_mode_callback(self, msg):
        self.current_mode = msg.mode
        self.get_logger().info('Set exo mode: %d' % self.current_mode)

    def state_estimation_callback(self, msg):
        self.current_left_foot_position = [msg.body_ankle_pose[0].position.x, msg.body_ankle_pose[0].position.y, msg.body_ankle_pose[0].position.z]
        self.current_right_foot_position = [msg.body_ankle_pose[1].position.x, msg.body_ankle_pose[1].position.y, msg.body_ankle_pose[1].position.z]
        self.dt = msg.step_time

        if self.current_mode == ExoMode.STAND:
            self.publish_iks_foot_positions(self.homestance_left_foot_position, self.homestance_right_foot_position)
        elif self.current_mode == ExoMode.VARIABLEWALK:
            if len(self.bezier_trajectory) == 0:
                return
            if self.current_swing_leg == 'LEFT':
                stance_leg_position = np.array(self.current_right_foot_position) * (self.leg_length / np.linalg.norm(np.array(self.current_right_foot_position)))
                self.publish_iks_foot_positions(self.bezier_trajectory.pop(0), self.homestance_left_foot_position)
            else:
                stance_leg_position = np.array(self.current_left_foot_position) * (self.leg_length / np.linalg.norm(np.array(self.current_left_foot_position)))
                self.publish_iks_foot_positions(self.homestance_right_foot_position, self.bezier_trajectory.pop(0))

    def desired_foot_positions_callback(self, msg):
        desired_foot_position = [msg.poses[0].position.x, msg.poses[0].position.y, msg.poses[0].position.z]
        if (msg.poses[0].position.y > msg.poses[125].position.y):
            self.current_swing_leg = 'LEFT'
            self.bezier_trajectory = self.create_bezier_trajectory(self.current_left_foot_position, desired_foot_position)
            self.get_logger().info('Swing leg is LEFT.')
        else:
            self.current_swing_leg = 'RIGHT'
            self.bezier_trajectory = self.create_bezier_trajectory(self.current_right_foot_position, desired_foot_position)
            self.get_logger().info('Swing leg is RIGHT.')
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'backpack'
        for foot_position in self.bezier_trajectory:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = 'backpack'
            pose_stamped.pose.position.x = foot_position[0]
            pose_stamped.pose.position.y = foot_position[1]
            pose_stamped.pose.position.z = foot_position[2]
            path.poses.append(pose_stamped)
        self.bezier_trajectory_publisher.publish(path)

    def publish_iks_foot_positions(self, left_foot_position, right_foot_position):
        iks_foot_positions_msg = IksFootPositions()
        iks_foot_positions_msg.header.stamp = self.get_clock().now().to_msg()
        iks_foot_positions_msg.header.frame_id = 'backpack'
        iks_foot_positions_msg.left_foot_position.x = left_foot_position[0]
        iks_foot_positions_msg.left_foot_position.y = left_foot_position[1]
        iks_foot_positions_msg.left_foot_position.z = left_foot_position[2]
        iks_foot_positions_msg.right_foot_position.x = right_foot_position[0]
        iks_foot_positions_msg.right_foot_position.y = right_foot_position[1]
        iks_foot_positions_msg.right_foot_position.z = right_foot_position[2]
        self.iks_foot_positions_publisher.publish(iks_foot_positions_msg)

    def create_bezier_trajectory(self, start_position, end_position):
        # Implement a Bezier curve trajectory that linearly raises the foot from the start position to a height of 0.2m and 0.5*x-position of end position, and then lowers it to the end position.
        ratio = 0.15
        height = 0.25
        peak_position = [
            start_position[0] + ratio * (end_position[0] - start_position[0]),
            start_position[1] + ratio * (end_position[1] - start_position[1]),
            start_position[2] + height
        ]
        T = 5.0 # s, duration of trajectory
        num_points = int(T/self.dt)
        self.get_logger().info(f'Starting trajectory from {start_position} to {peak_position} to {end_position}.')
        starting_trajectory = np.linspace(np.array(start_position), np.array(peak_position), int(ratio * num_points))
        ending_trajectory = np.linspace(np.array(peak_position), np.array(end_position), int((1-ratio) * num_points))
        trajectory = np.concatenate((starting_trajectory, ending_trajectory))
        return trajectory.tolist()


def main(args=None):
    rclpy.init(args=args)
    mpc_planning_node = MpcPlanningNode()
    rclpy.spin(mpc_planning_node)
    mpc_planning_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
