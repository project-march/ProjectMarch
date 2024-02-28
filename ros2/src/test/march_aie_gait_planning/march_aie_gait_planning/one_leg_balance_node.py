import rclpy
from rclpy.node import Node

from march_shared_msgs.msg import IksFootPositions, StateEstimation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import numpy as np

AIE_OSCILLATION_DEGREE = 5.0 * np.pi / 180.0

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

        self.joint_trajectory_publisher = self.create_publisher(
            JointTrajectory,
            'joint_trajectory_controller/joint_trajectory',
            10
        )

        self.current_state = StateEstimation()        
        self.current_stance_leg = 0b11
        self.desired_stance_leg = 0b11
        self.step_height = 0.35
        self.step_width = 0.06
        self.no_of_points = 400
        self.counter = 0
        self.state = 'standing'
        self.is_raised = False
        
        self.home_stance = self.generate_home_stance()
        self.raising_trajectory, self.raise_stance = self.generate_raising_trajectory()
        self.states = {
            'standing': {
                'counter': 100,
                'transition': 'raising',
            },
            'raising': {
                'counter': self.no_of_points,
                'transition': 'balancing',
            },
            'balancing': {
                'counter': self.no_of_points,
                'transition': 'balancing',
            },
        }

        self.get_logger().info('March AIE Gait Planning - One Leg Balance Node Started')

    def state_estimation_callback(self, msg):
        if not self.is_raised:
            self.current_state = msg
        self.publish_iks_foot_positions()

    def publish_iks_foot_positions(self):
        if self.state == 'standing':
            msg = self.home_stance
            msg.header.stamp = self.get_clock().now().to_msg()
            self.iks_foot_position_publisher.publish(msg)
        elif self.state == 'raising':
            msg = self.raising_trajectory[self.counter]
            msg.header.stamp = self.get_clock().now().to_msg()
            self.iks_foot_position_publisher.publish(msg)
        # elif self.state == 'balancing':
        #     msg = JointTrajectory()
        #     msg.header.stamp = self.get_clock().now().to_msg()
        #     msg.joint_names = ['left_ankle', 'left_hip_aa', 'left_hip_fe', 'left_knee', 'right_ankle', 'right_hip_aa', 'right_hip_fe', 'right_knee']


        #     # current_point = JointTrajectoryPoint()
        #     # current_point.positions = self.current_state.joint_state.position
        #     # current_point.velocities = self.current_state.joint_state.velocity
        #     # current_point.time_from_start.sec = 0
        #     # current_point.time_from_start.nanosec = 0

        #     next_point = JointTrajectoryPoint()
        #     next_point.positions = self.current_state.joint_state.position
        #     next_point.positions[5] = next_point.positions[5] + AIE_OSCILLATION_DEGREE * np.sin(self.counter * np.pi / self.no_of_points)
        #     next_point.velocities = self.current_state.joint_state.velocity
        #     next_point.accelerations = [0.0] * len(next_point.positions)
        #     next_point.effort = [0.0] * len(next_point.positions)
        #     next_point.time_from_start.sec = 0
        #     next_point.time_from_start.nanosec = 5000000

        #     msg.points = [next_point]
        #     self.joint_trajectory_publisher.publish(msg)
        else:
            msg = self.raise_stance
            msg.header.stamp = self.get_clock().now().to_msg()
            self.iks_foot_position_publisher.publish(msg)

        if self.counter == self.states[self.state]['counter'] - 1:
            if self.state == 'raising':
                self.is_raised = True
            self.state = self.states[self.state]['transition']
            self.counter = 0
        else:
            self.counter += 1

    def generate_home_stance(self):
        home_stance = IksFootPositions()
        home_stance.left_foot_position.x = 0.36
        home_stance.left_foot_position.y = 0.12
        home_stance.left_foot_position.z = -0.7
        home_stance.right_foot_position.x = 0.36
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

def main(args=None):
    rclpy.init(args=args)
    one_leg_balance_node = OneLegBalanceNode()
    rclpy.spin(one_leg_balance_node)
    one_leg_balance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()