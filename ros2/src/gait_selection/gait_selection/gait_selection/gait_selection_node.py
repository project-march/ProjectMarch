"""Author: Marco Bak MVIII."""
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from march_shared_msgs.srv import RequestGait

from gait_selection.gait_loader import GaitLoader


class GaitSelectionNode(Node):
    """This gait is there to select the gaits that are not execute in a balanced fashion.

    For now these gaits are:
    - Sitting down;
    - Standing up.
    """

    def __init__(self):
        super().__init__('gait_selection_node')
        self.gait_package = "gait_files"
        self.directory_name = "airgait_vi"
        self.gait_loader = GaitLoader(self)
        self.publisher_ = self.create_publisher(JointTrajectory, 'joint_trajectory_controller/joint_trajectory', 10)

        self.service = self.create_service(RequestGait, "gait_selection", self.service_callback)

    def service_callback(self, request, response):
        """The callback that sends the requested gait to the joint_trajectory_controller/joint_trajectory topic."""
        requested_gait = request.gait_type
        gait = None
        if requested_gait == 0:
            self.get_logger().warn("sit gait called!")
            gait = self.gait_loader.loaded_gaits.get("stand")
        elif requested_gait == 1:
            self.get_logger().warn("stand gait called!")
            gait = self.gait_loader.loaded_gaits["sit"]
        msg = gait.start(self.get_clock().now()).new_trajectory_command.trajectory
        self.get_logger().debug(str(msg))
        self.publisher_.publish(msg)
        response.status = True
        return response


def main(args=None):
    """Main function that spins the node."""
    rclpy.init(args=args)

    gait_selection_node = GaitSelectionNode()

    rclpy.spin(gait_selection_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gait_selection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
