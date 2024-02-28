import rclpy
from rclpy.node import Node

from march_shared_msgs.msg import IksFootPositions, StateEstimation

import numpy as np

class MarchAieGaitPlanningNode(Node):

    def __init__(self):
        super().__init__('march_aie_gait_planning_node')

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
        self.is_standing = False # TODO: Implement a standing check
        
        self.home_stance = self.generate_home_stance()
        self.gait_trajectory = self.generate_marching_gait_trajectory()

        self.get_logger().info('March AIE Gait Planning Node Started')

    def state_estimation_callback(self, msg):
        self.current_stance_leg = msg.stance_leg
        self.publish_iks_foot_positions()

    def publish_iks_foot_positions(self):
        msg = self.gait_trajectory[self.counter]
        msg.header.stamp = self.get_clock().now().to_msg()
        self.iks_foot_position_publisher.publish(msg)
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
    
    def generate_marching_gait_trajectory(self):
        gait_trajectory = []
        y_trajectory_inverse = np.linspace(self.home_stance.left_foot_position.y, self.home_stance.left_foot_position.y - self.step_width, self.no_of_points // 4)
        y_trajectory_inverse = np.concatenate((y_trajectory_inverse, y_trajectory_inverse[::-1]))
        y_trajectory_everse = np.linspace(self.home_stance.left_foot_position.y, self.home_stance.left_foot_position.y + self.step_width / 2, self.no_of_points // 4)
        y_trajectory_everse = np.concatenate((y_trajectory_everse, y_trajectory_everse[::-1]))
        z_trajectory = np.linspace(self.home_stance.left_foot_position.z, self.home_stance.left_foot_position.z + self.step_height, self.no_of_points // 4)
        z_trajectory = np.concatenate((z_trajectory, z_trajectory[::-1]))
        z_trajectory = np.concatenate((z_trajectory, z_trajectory))

        for i in range(self.no_of_points):
            gait_positions = IksFootPositions()
            gait_positions.left_foot_position.x = self.home_stance.left_foot_position.x
            gait_positions.right_foot_position.x = self.home_stance.right_foot_position.x

            if i < self.no_of_points // 2:
                gait_positions.left_foot_position.y = y_trajectory_everse[i]
                gait_positions.left_foot_position.z = z_trajectory[i]
                gait_positions.right_foot_position.y = -y_trajectory_inverse[i]
                gait_positions.right_foot_position.z = self.home_stance.right_foot_position.z
            else:
                gait_positions.left_foot_position.y = y_trajectory_inverse[i//2]
                gait_positions.left_foot_position.z = self.home_stance.left_foot_position.z
                gait_positions.right_foot_position.y = -y_trajectory_everse[i//2]
                gait_positions.right_foot_position.z = z_trajectory[i]
            gait_trajectory.append(gait_positions)
        return gait_trajectory
    
    def multiply_trajectory(self, trajectory):
        trajectory = np.concatenate((trajectory, trajectory[::-1]))
        trajectory = np.concatenate((trajectory, trajectory))
        return trajectory

def main(args=None):
    rclpy.init(args=args)
    march_aie_gait_planning_node = MarchAieGaitPlanningNode()
    rclpy.spin(march_aie_gait_planning_node)
    march_aie_gait_planning_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()