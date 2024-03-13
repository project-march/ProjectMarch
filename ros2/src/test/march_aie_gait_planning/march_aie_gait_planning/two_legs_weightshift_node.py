import rclpy
from rclpy.node import Node

from march_shared_msgs.msg import IksFootPositions, StateEstimation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import numpy as np

AIE_OSCILLATION_DEGREE = 5.0 * np.pi / 180.0

class TwoLegsWeightShiftNode(Node):

    def __init__(self):
        super().__init__('two_legs_weightshift_node')

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

        self.joint_trajectory_publisher = self.create_publisher(
            JointTrajectory,
            'joint_trajectory_controller/joint_trajectory',
            10
        )

        self.current_state = StateEstimation()        
        self.current_stance_leg = 0b11
        self.desired_stance_leg = 0b11
        self.step_height = 0.02
        self.step_width = 0.02
        self.no_of_points = 800
        self.counter = 0
        self.state = 'standing'
        
        self.home_stance = self.generate_home_stance()
        self.weightshifting_gait = self.generate_weightshifting_trajectory()
        self.states = {
            'standing': {
                'counter': 100,
                'transition': 'weightshifting',
            },
            'weightshifting': {
                'counter': self.no_of_points,
                'transition': 'weightshifting',
            },
        }

        self.get_logger().info('March AIE Gait Planning - Two Legs Weightshift Node has been initialized')

    def state_estimation_callback(self, msg):
        self.publish_iks_foot_positions()

    def publish_iks_foot_positions(self):
        if self.state == 'weightshifting':
            msg = self.weightshifting_gait[self.counter]
            msg.header.stamp = self.get_clock().now().to_msg()
            self.iks_foot_position_publisher.publish(msg)
        else:
            msg = self.home_stance
            msg.header.stamp = self.get_clock().now().to_msg()
            self.iks_foot_position_publisher.publish(msg)

        if self.counter == self.states[self.state]['counter'] - 1:
            self.state = self.states[self.state]['transition']
            self.counter = 0
        else:
            self.counter += 1

    def generate_home_stance(self):
        home_stance = IksFootPositions()
        home_stance.left_foot_position.x = 0.36
        home_stance.left_foot_position.y = 0.16
        home_stance.left_foot_position.z = -0.7
        home_stance.right_foot_position.x = 0.36
        home_stance.right_foot_position.y = -0.16
        home_stance.right_foot_position.z = -0.7
        return home_stance

    def generate_weightshifting_trajectory(self):
        gait_trajectory = []
        y_trajectory_inverse = np.linspace(self.home_stance.right_foot_position.y, self.home_stance.right_foot_position.y + self.step_width, self.no_of_points // 4)
        y_trajectory_everse = np.linspace(self.home_stance.left_foot_position.y, self.home_stance.left_foot_position.y + self.step_width, self.no_of_points // 4)
        y_trajectory_inverse = np.concatenate([y_trajectory_inverse, np.flip(y_trajectory_inverse)])
        y_trajectory_everse = np.concatenate([y_trajectory_everse, np.flip(y_trajectory_everse)])
        z_trajectory = np.linspace(self.home_stance.right_foot_position.z, self.home_stance.right_foot_position.z + self.step_height, self.no_of_points // 4)
        z_trajectory = np.concatenate([z_trajectory, np.flip(z_trajectory)])

        for y_inv, y_ev, z in zip(y_trajectory_inverse, y_trajectory_everse, z_trajectory):
            gait_positions = IksFootPositions()
            gait_positions.left_foot_position.x = self.home_stance.left_foot_position.x
            gait_positions.left_foot_position.y = y_ev
            gait_positions.left_foot_position.z = z
            gait_positions.right_foot_position.x = self.home_stance.right_foot_position.x
            gait_positions.right_foot_position.y = y_inv
            gait_positions.right_foot_position.z = self.home_stance.right_foot_position.z
            gait_trajectory.append(gait_positions)
        
        for y_inv, y_ev, z in zip(y_trajectory_inverse, y_trajectory_everse, z_trajectory):
            gait_positions = IksFootPositions()
            gait_positions.left_foot_position.x = self.home_stance.left_foot_position.x
            gait_positions.left_foot_position.y = -y_inv
            gait_positions.left_foot_position.z = self.home_stance.left_foot_position.z
            gait_positions.right_foot_position.x = self.home_stance.right_foot_position.x
            gait_positions.right_foot_position.y = -y_ev
            gait_positions.right_foot_position.z = z
            gait_trajectory.append(gait_positions)

        return gait_trajectory


def main(args=None):
    rclpy.init(args=args)
    two_legs_weightshift_node = TwoLegsWeightShiftNode()
    rclpy.spin(two_legs_weightshift_node)
    two_legs_weightshift_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()