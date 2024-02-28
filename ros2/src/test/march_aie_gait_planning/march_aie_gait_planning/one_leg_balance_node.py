import rclpy
from rclpy.node import Node

from march_shared_msgs.msg import IksFootPositions, StateEstimation

import numpy as np

class OneLegBalanceNode(Node):

    def __init__(self):
        super().__init__('one_leg_balance_node')

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
        self.step_height = 0.25
        self.step_width = 0.06
        self.no_of_points = 200
        self.counter = 0
        self.state = 'standing'
        
        self.home_stance = self.generate_home_stance()
        self.raising_trajectory, self.raise_stance = self.generate_raising_trajectory()
        self.state_transitions = {
            'standing': 'raising',
            'raising': 'balancing',
            'balancing': 'balancing',
        }

        self.get_logger().info('March AIE Gait Planning - One Leg Balance Node Started')

    def state_estimation_callback(self, msg):
        self.current_stance_leg = msg.stance_leg
        self.publish_iks_foot_positions()

    def publish_iks_foot_positions(self):
        if self.state == 'standing':
            msg = self.home_stance
        elif self.state == 'raising':
            msg = self.raising_trajectory[self.counter]
        else:
            msg = self.raise_stance
        msg.header.stamp = self.get_clock().now().to_msg()
        self.iks_foot_position_publisher.publish(msg)
        
        if self.counter == self.no_of_points - 1:
            self.state = self.state_transitions[self.state]
        self.counter = (self.counter + 1) % self.no_of_points

    def generate_home_stance(self):
        home_stance = IksFootPositions()
        home_stance.left_foot_position.x = 0.3
        home_stance.left_foot_position.y = 0.12
        home_stance.left_foot_position.z = -0.7
        home_stance.right_foot_position.x = 0.3
        home_stance.right_foot_position.y = -0.12
        home_stance.right_foot_position.z = -0.7
        return home_stance
    
    def generate_raising_trajectory(self):
        gait_trajectory = []
        y_trajectory_inverse = np.linspace(self.home_stance.right_foot_position.y, self.home_stance.right_foot_position.y + self.step_width, self.no_of_points)
        y_trajectory_everse = np.linspace(self.home_stance.left_foot_position.y, self.home_stance.left_foot_position.y + self.step_width, self.no_of_points)
        z_trajectory = np.linspace(self.home_stance.left_foot_position.z, self.home_stance.left_foot_position.z + self.step_height, self.no_of_points)

        final_foot_positions = self.home_stance
        for y_inv, y_ev, z in zip(y_trajectory_inverse, y_trajectory_everse, z_trajectory):
            gait_positions = IksFootPositions()
            gait_positions.left_foot_position.x = self.home_stance.left_foot_position.x
            gait_positions.left_foot_position.y = y_ev
            gait_positions.left_foot_position.z = z
            gait_positions.right_foot_position.x = self.home_stance.right_foot_position.x
            gait_positions.right_foot_position.y = y_inv
            gait_positions.right_foot_position.z = self.home_stance.right_foot_position.z
            gait_trajectory.append(gait_positions)
            final_foot_positions = gait_positions
        return gait_trajectory, final_foot_positions
    
    def generate_one_leg_balance_trajectory(self):
        gait_trajectory = []
        for _ in range(self.no_of_points):
            gait_trajectory.append(self.home_stance)
        return gait_trajectory

def main(args=None):
    rclpy.init(args=args)
    one_leg_balance_node = OneLegBalanceNode()
    rclpy.spin(one_leg_balance_node)
    one_leg_balance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()