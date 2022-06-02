"""Author: ???."""

from __future__ import annotations

from queue import Queue, LifoQueue
from typing import List, Set, Optional

from attr import dataclass
from sensor_msgs.msg import JointState

from control_msgs.action._follow_joint_trajectory import FollowJointTrajectory_GetResult_Response, \
    FollowJointTrajectory_FeedbackMessage, FollowJointTrajectory_Feedback
from rclpy import Future
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle

from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from march_utility.gait.subgait import Subgait
from march_utility.utilities.duration import Duration
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalID
from march_shared_msgs.msg import (
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryActionGoal,
    FollowJointTrajectoryActionResult,
    FollowJointTrajectoryResult,
)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

TRAJECTORY_SCHEDULER_HISTORY_DEPTH = 5


@dataclass
class TrajectoryCommand:
    """A container for scheduling trajectories.

    It contains besides the trajectory to be scheduled some additional information
    about the scheduled trajectory, such as the subgait name, duration and start time
    of the trajectory.
    """

    trajectory: JointTrajectory
    duration: Duration
    name: str
    start_time: Time

    @staticmethod
    def from_subgait(subgait: Subgait, start_time: Time) -> TrajectoryCommand:
        """Create a TrajectoryCommand from a subgait.

        Args:
            subgait (Subgait): subgait to create a command from
            start_time (Time): time at which subgait should be scheduled
        Returns:
            TrajectoryCommand: command corresponding to the given subgait at the given start time
        """
        return TrajectoryCommand(
            subgait.to_joint_trajectory_msg(),
            subgait.duration,
            subgait.subgait_name,
            start_time,
        )

    def __str__(self) -> str:
        """Returns a string with the name, start time and duration."""
        return f"({self.name}, {self.start_time.nanoseconds}, {self.duration.nanoseconds})"


class TrajectoryScheduler:
    """Scheduler that sends the wanted trajectories to the topic listened to by the exoskeleton/simulation.

    Args:
        node (Node): node that is used to create subscribers/publishers
    Attributes:
        _logger (Logger): used to log to the terminal
        _failed (bool): ???
        _node (Node): node that is used to create subscribers/publishers
        _goals (List[TrajectoryCommand]): list containing trajectory commands
        _trajectory_goal_pub (Publisher): used to publish FollowJointTrajectoryActionGoal on
            /march/controller/trajectory/follow_joint_trajectory_goal
        _cancel_pub (Publisher): publishes a GoalID message on /march/controller/trajectory/follow_joint...
            ..._trajectory/goal
        _trajectory_goal_result_sub (Subscriber): Listens for FollowJointTrajectoryActionResult on
            "march/controller/trajectory/follow_joint_trajectory/result. Receives the errors
        _trajectory_command_pub (Publisher): publishes JointTrajectory messages on
            /march/controller/trajectory/command

        For more information on the Action client see these pages:
            * https://docs.ros.org/en/foxy/Tutorials/Understanding-ROS2-Actions.html?highlight=action
            * https://docs.ros.org/en/foxy/Tutorials/Actions/Creating-an-Action.html
            * https://docs.ros.org/en/foxy/Tutorials/Actions/Writing-a-Py-Action-Server-Client.html
    """

    def __init__(self, node: Node):
        self._failed = False
        self._node = node
        # self._goals: List[TrajectoryCommand] = []
        # self._active_goals: Set[Future | ClientGoalHandle] = set()
        self._active_goals = LifoQueue()
        self._messages_in_transit: Set[Future] = set()
        self._last_position = []
        self._logger = node.get_logger().get_child(__class__.__name__)
        self._schedule_timeout = node.get_parameter("early_schedule_duration").get_parameter_value().double_value

        # Temporary solution to communicate with ros1 action server, should
        # be updated to use ros2 action implementation when simulation is
        # migrated to ros2
        # self._trajectory_goal_pub = self._node.create_publisher(
        #     msg_type=FollowJointTrajectoryActionGoal,
        #     topic="/march/controller/trajectory/follow_joint_trajectory/goal",
        #     qos_profile=TRAJECTORY_SCHEDULER_HISTORY_DEPTH,
        # )

        # self._cancel_pub = self._node.create_publisher(
        #     msg_type=GoalID,
        #     topic="/march/controller/trajectory/follow_joint_trajectory/cancel",
        #     qos_profile=TRAJECTORY_SCHEDULER_HISTORY_DEPTH,
        # )
        #
        # self._trajectory_goal_result_sub = self._node.create_subscription(
        #     msg_type=FollowJointTrajectoryActionResult,
        #     topic="/march/controller/trajectory/follow_joint_trajectory/result",
        #     callback=self._done_cb,
        #     qos_profile=TRAJECTORY_SCHEDULER_HISTORY_DEPTH,
        # )

        # Publisher for sending hold position mode
        # self._trajectory_command_pub = self._node.create_publisher(
        #     msg_type=JointTrajectory,
        #     topic="/march/controller/trajectory/command",
        #     qos_profile=TRAJECTORY_SCHEDULER_HISTORY_DEPTH,
        # )
        self.action_client = ActionClient(self._node, FollowJointTrajectory,
                                          "/joint_trajectory_controller/follow_joint_trajectory")
        # self._node.create_subscription(JointState, "/joint_states", self.print_val, 1)

    def schedule(self, command: TrajectoryCommand) -> None:
        """Schedules a new trajectory.

        Args:
            command (TrajectoryCommand): The trajectory command to schedule
        """
        self._failed = False
        joints = command.trajectory.joint_names
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = command.trajectory
        goal_msg.goal_tolerance = [JointTolerance(position=0.0001)] * len(joints)
        if not self.action_client.wait_for_server(self._schedule_timeout):
            self._logger.warn(f"Failed to schedule trajectory {command} within {self._schedule_timeout} seconds")
            return

        # self._node.get_logger().info(f"Starting the goal, for joint names: {joints}, to positions: {command.trajectory.points}")
        goal_future: Future = self.action_client.send_goal_async(goal_msg)
        goal_future.add_done_callback(self._gait_executor_start_request_cb)
        self._messages_in_transit.add(goal_future)

        # region Old code
        # stamp = command.start_time.to_msg()
        # command.trajectory.header.stamp = stamp
        # goal = FollowJointTrajectoryGoal(trajectory=command.trajectory)
        # self._trajectory_goal_pub.publish(
        #     FollowJointTrajectoryActionGoal(
        #         header=Header(stamp=stamp),
        #         goal_id=GoalID(stamp=stamp, id=str(command)),
        #         goal=goal,
        #     )
        # )
        # info_log_message = f"Scheduling {command.name}"
        # debug_log_message = f"Subgait {command.name} starts "
        # if self._node.get_clock().now() < command.start_time:
        #     time_difference = Duration.from_ros_duration(command.start_time - self._node.get_clock().now())
        #     debug_log_message += f"in {round(time_difference.seconds, 3)}s"
        # else:
        #     debug_log_message += "now"
        #
        # self._goals.append(command)
        # self._logger.info(info_log_message)
        # self._logger.debug(debug_log_message)
        # endregion

    # def _store_last_joint_state_cb(self, msg: JointState):
    #     self._last_position = msg.position

    # def print_val(self, val: FollowJointTrajectory_FeedbackMessage):
    #     self._last_position = val.feedback
    #     # val.feedback
    #     # self._logger.info(f"Val of feedback: (Type: {type(val)}, {val})")
    #     pass

    def _gait_executor_start_request_cb(self, future: Future):
        self._messages_in_transit.remove(future)
        gait_executor_response_goal_handle: ClientGoalHandle = future.result()
        if not gait_executor_response_goal_handle.accepted:
            self._logger.warning("Goal message to execute gait was not accepted by the Action server (e.g. gazebo). "
                                 f"Goal: {gait_executor_response_goal_handle}")
            return

        self._active_goals.put(future)
        self._logger.info(f"Goal message to execute gait is accepted.")
        gait_execution_result_future: Future = gait_executor_response_goal_handle.get_result_async()
        gait_execution_result_future.add_done_callback(self._gait_executor_finished_cb)

    def _gait_executor_finished_cb(self, future: Future):
        """

        Args:
            future (Future[FollowJointTrajectory_GetResult_Response]):

        Returns:

        """
        result: FollowJointTrajectoryResult = future.result().result
        if result.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
            self._logger.info(f"Successfully executed gait: {type(future.result())}")
        else:
            self._logger.warning(f"Failed to execute trajectory: {result.error_string}")
            self._failed = True

    def cancel_active_goals(self) -> None:
        """Cancels the active goal."""
        # self._logger.info(f"---------------------------------------Cancel active goals: {self._messages_in_transit}")
        # self._logger.info(f"---------------------------------------Cancel active goals: {self._active_goals}")
        for msg in self._messages_in_transit:
            msg.cancel()
        while not self._active_goals.empty():
            self._active_goals.get().result().cancel_goal_async()


    def send_position_hold(self) -> None:
        self._logger.info("Holding.....................")
        # last_trajectory = self._last_position.get_fields_and_field_types()
        # last_position: JointTrajectoryPoint = last_trajectory.get("actual")
        # last_position.get_fields_and_field_types()


        # hold_command = TrajectoryCommand(
        #     trajectory=JointTrajectory(
        #         header=last_position.get("header"),
        #         joint_names=last_position.get("joint_names"),
        #         points=last_position.get("actual")
        #     ),
        #     duration=Duration(0),
        #     name="Hold position",
        #     start_time=Time()
        # )
        # v = self._last_trajectory.
        self._logger.info(f"Holding..................... ({5})")

        # pass
        """Schedule empty JointTrajectory message to hold position. Used during force unknown."""
        # self.schedule(TrajectoryCommand(trajectory=JointTrajectory()))
        # pass
        # self._trajectory_command_pub.publish(JointTrajectory())

    def failed(self) -> bool:
        """Returns true if the trajectory failed."""
        return self._failed

    def reset(self) -> None:
        """Reset attributes of class."""
        self._failed = False
        self._active_goals = LifoQueue()
        # self._goals = []

    # def _done_cb(self, result) -> None:
    #     """Callback for when a result is published."""
    #     if result.result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
    #         self._logger.error(
    #             f"Failed to execute trajectory. {result.result.error_string} ({result.result.error_code})"
    #         )
    #         self._failed = True
