"""Author: Marco Bak MVIII."""
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Int32, Bool
from march_shared_msgs.srv import RequestGait

from rclpy import Future
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle

from control_msgs.action import FollowJointTrajectory

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
        self.joint_names = [
            "left_ankle", "left_hip_aa", "left_hip_fe", "left_knee",
            "right_ankle", "right_hip_aa", "right_hip_fe", "right_knee"]
        self.gait_loader = GaitLoader(self)
        self.publisher_ = self.create_publisher(JointTrajectory, 'joint_trajectory_controller/joint_trajectory', 10)

        self._action_client_to_controller = ActionClient(
            self, FollowJointTrajectory, "/joint_trajectory_controller/follow_joint_trajectory"
        )

        self.reset_publisher = self.create_publisher(Int32, "/trajectory_reset_gate", 10)
        self.sim_reset_publisher = self.create_publisher(Bool, "/mujoco_reset_trajectory", 10)
        self.actual_joint_state = [0, 0, 0, 0, 0, 0, 0, 0]

        self.get_logger().info(str(self.gait_loader.loaded_gaits))
        self._gait_executed = None

        self.service = self.create_service(RequestGait, "gait_selection", self.service_callback)

    def service_callback(self, request, response):
        """The callback that sends the requested gait to the joint_trajectory_controller/joint_trajectory topic."""
        requested_gait = request.gait_type

        gait = None
        reset_msg = Int32()

        if requested_gait == 0:
            self.get_logger().warn("sit gait called!")

            gait = self.gait_loader.loaded_gaits.get("home_sit")
            trajectory = gait.start(self.get_clock().now()).new_trajectory_command.trajectory
            self._execute_gait(trajectory)
            reset_msg.data = -1
            self.reset_publisher.publish(reset_msg)
        elif requested_gait == 1:
            self.get_logger().warn("stand gait called!")
            gait = self.gait_loader.loaded_gaits["home_stand"]
            trajectory = gait.start(self.get_clock().now()).new_trajectory_command.trajectory
            self._execute_gait(trajectory)

        # Used to make sure the sim plans the gaits in time.
        sim_reset_msg = Bool()
        sim_reset_msg.data = True
        self.sim_reset_publisher.publish(sim_reset_msg)

        response.status = True
        return response

    def _execute_gait(self, trajectory):
        """Execute the gait with the action server of the joint trajectory controller.

        :param trajectory:
        :return:
        """
        self._gait_executed = False
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        self.get_logger().info(str(goal_msg.trajectory))
        goal_future: Future = self._action_client_to_controller.send_goal_async(goal_msg)
        goal_future.add_done_callback(self._send_goal_callback)

    def _send_goal_callback(self, future: Future) -> None:
        """Callback for if the trajectory start_request message has arrived at the controller.

        It will check if the trajectory is accepted:
            * If accepted it will store the goal_handle and start a call to `_controller_finished_trajectory_cb(future)`
                when the trajectory is done executing.
            * If not accepted, it will log a warning and do nothing. The trajectory will not be executed because
                it is canceled from the controllers side.

        Args:
            future (Future[ClientGoalHandle]): Finished future object containing as result an ClientGoalHandle.
        """
        gait_executor_response_goal_handle: ClientGoalHandle = future.result()
        if not gait_executor_response_goal_handle.accepted:
            self.get_logger().warning(
                "Goal message to execute gait was not accepted by the Action server."
                f"Goal: {gait_executor_response_goal_handle}"
            )
            return
        self._logger.info("Goal message to execute gait is accepted.")
        gait_execution_result_future: Future = gait_executor_response_goal_handle.get_result_async()
        gait_execution_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future: Future) -> None:
        """Callback for if the trajectory is finished.

        If the trajectory did not execute successfully it will set the `_failed` value to True, which can later be
        retrieved by the statemachine.

        Args:
            future (Future[FollowJointTrajectory_GetResult_Response]): A future object containing the result if
                the trajectory is successfully executed.
        """
        if future.result().result.error_code != future.result().result.SUCCESSFUL:
            self.get_logger().error(
                "Failed to execute trajectory:" + str(future.result().result.error_code.error_string))
            self._gait_executed = True
        else:
            self.get_logger().info("Trajectory executed correctly.")
            self._gait_executed = True


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
