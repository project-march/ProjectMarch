"""Author: Marco Bak MVIII."""
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from march_shared_msgs.srv import RequestGait, RequestFootsteps

from test_setup_gait_selection.gait_loader import GaitLoader


class TestSetupGaitSelectionNode(Node):
    """This gait is there to select the gaits that are not execute in a balanced fashion.

    For now these gaits are:
    - Sitting down;
    - Standing up.
    """

    def __init__(self):
        super().__init__('TestSetupGaitSelectionNode')

        self.declare_parameter("test_rotational")
        test_rotational = self.get_parameter("test_rotational").get_parameter_value().bool_value

        self.gait_package = "gait_files"
        self.directory_name = "test_joint_linear_gaits"
        self.joints = ["linear_joint"]
        if test_rotational:
            self.directory_name = "test_joint_rotational_gaits"
            self.joints = ["rotational_joint"]

        self.gait_loader = GaitLoader(self)
        self.get_logger().info("Joint is: " + str(self.joints))
        self.get_logger().info("Possible gaits are: " + str(self.gait_loader.gaits))
        self.get_logger().info("Possible gaits are: " + str(self.gait_loader._named_positions.values()))

        self.publisher_ = self.create_publisher(JointTrajectory, 'joint_trajectory_controller/joint_trajectory', 10)

        self.gait_service = self.create_service(RequestGait, "gait_selection", self.gait_service_callback)
        self.service = self.create_service(RequestFootsteps, "footstep_generator", self.footstep_service_callback)

    def gait_service_callback(self, request, response):
        """The callback that sends the requested gait to the joint_trajectory_controller/joint_trajectory topic."""
        requested_gait = request.gait_type
        gait = None
        self.get_logger().info("Possible gaits are: " + str(self.gait_loader.gaits))
        if requested_gait == 0:
            self.get_logger().info("home_setup called!")
            gait = self.gait_loader.gaits["home_setup"]
        elif requested_gait == 1:
            self.get_logger().info("test_joint_gait called!")
            gait = self.gait_loader.gaits["test_joint_gait"]
        msg = gait.start(self.get_clock().now()).new_trajectory_command.trajectory
        self.get_logger().info(str(msg))
        self.publisher_.publish(msg)
        response.status = True
        return response

    def footstep_service_callback(self, request, response):
        """Response callback for the footstep_planner, this should not be used in the test-setup."""
        response.status = True
        return response


def main(args=None):
    """Main function that spins the node."""
    rclpy.init(args=args)

    test_setup_git_selection_node = TestSetupGaitSelectionNode()

    rclpy.spin(test_setup_git_selection_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test_setup_git_selection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
