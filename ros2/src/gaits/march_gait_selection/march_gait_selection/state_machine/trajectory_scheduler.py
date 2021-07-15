from typing import List, Dict

import rclpy
from actionlib_msgs.msg import GoalID
from attr import dataclass
from march_shared_msgs.msg import (
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryActionGoal,
    FollowJointTrajectoryActionResult,
    FollowJointTrajectoryResult,
)
from march_shared_msgs.srv import GetParamStringList
from march_utility.gait.subgait import Subgait
from march_utility.utilities.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

POSSIBLE_CONTROLLER_NAMES = [
    "trajectory",
    "trajectory_mpc",
]


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
    def from_subgait(subgait: Subgait, start_time: Time):
        """Create a TrajectoryCommand from a subgait."""
        return TrajectoryCommand(
            subgait.to_joint_trajectory_msg(),
            subgait.duration,
            subgait.subgait_name,
            start_time,
        )

    def __str__(self):
        return (
            f"({self.name}, {self.start_time.nanoseconds}, {self.duration.nanoseconds})"
        )


# Typing hint
ControllerName = str


class TrajectoryScheduler:
    """Scheduler that sends sends the wanted trajectories to the topic listened
    to by the exoskeleton/simulation."""

    def __init__(self, node: Node):
        self._failed = False
        self._node = node
        self._goals: List[TrajectoryCommand] = []

        self._active_controller_names: List[ControllerName] = []

        self._trajectory_goal_pubs: Dict[ControllerName, Publisher] = {}
        self._cancel_pubs: Dict[ControllerName, Publisher] = {}
        self._trajectory_goal_result_subs: Dict[ControllerName, Subscription] = {}
        self._trajectory_command_pubs: Dict[ControllerName, Publisher] = {}

        for controller_name in POSSIBLE_CONTROLLER_NAMES:
            if (
                self._node.count_subscribers(
                    f"/march/controller/{controller_name}/follow_joint_trajectory/goal"
                )
                == 0
            ):
                # If the topic does not exist, then this controller is not active
                continue

            self._active_controller_names.append(controller_name)

            # Temporary solution to communicate with ros1 action server, should
            # be updated to use ros2 action implementation when simulation is
            # migrated to ros2
            self._trajectory_goal_pubs[controller_name] = self._node.create_publisher(
                msg_type=FollowJointTrajectoryActionGoal,
                topic=f"/march/controller/{controller_name}/follow_joint_trajectory/goal",
                qos_profile=5,
            )

            self._cancel_pubs[controller_name] = self._node.create_publisher(
                msg_type=GoalID,
                topic=f"/march/controller/{controller_name}/follow_joint_trajectory/cancel",
                qos_profile=5,
            )

            self._trajectory_goal_result_subs[
                controller_name
            ] = self._node.create_subscription(
                msg_type=FollowJointTrajectoryActionResult,
                topic=f"/march/controller/{controller_name}/follow_joint_trajectory/result",
                callback=self._done_cb,
                qos_profile=5,
            )

            # Publisher for sending hold position mode
            self._trajectory_command_pubs[
                controller_name
            ] = self._node.create_publisher(
                msg_type=JointTrajectory,
                topic=f"/march/controller/{controller_name}/command",
                qos_profile=5,
            )

        if self.uses_mixed_control:
            # This member is only used when uses_mixed_control is True
            self._joints_per_controller = self.get_joint_names_per_controller()

    @property
    def uses_mixed_control(self) -> bool:
        """Is mixed control (multiple controllers) used?"""
        return len(self._active_controller_names) > 1

    def schedule(self, command: TrajectoryCommand):
        """Schedules a new trajectory.
        :param TrajectoryCommand command: The trajectory command to schedule
        """
        self._failed = False
        stamp = command.start_time.to_msg()
        command.trajectory.header.stamp = stamp

        for controller_name, publisher in self._trajectory_goal_pubs.items():
            if self.uses_mixed_control:
                trajectory = self.extract_joint_trajectory_by_joint_names(
                    command.trajectory, self._joints_per_controller[controller_name]
                )
            else:
                trajectory = command.trajectory
            goal = FollowJointTrajectoryGoal(trajectory=trajectory)
            publisher.publish(
                FollowJointTrajectoryActionGoal(
                    header=Header(stamp=stamp),
                    goal_id=GoalID(stamp=stamp, id=str(command)),
                    goal=goal,
                )
            )

        info_log_message = f"Scheduling {command.name}"
        debug_log_message = f"Subgait {command.name} starts "
        if self._node.get_clock().now() < command.start_time:
            time_difference = Duration.from_ros_duration(
                command.start_time - self._node.get_clock().now()
            )
            debug_log_message += f"in {round(time_difference.seconds, 3)}s"
        else:
            debug_log_message += "now"

        self._goals.append(command)
        self._node.get_logger().info(info_log_message)
        self._node.get_logger().debug(debug_log_message)

    def cancel_active_goals(self):
        now = self._node.get_clock().now()
        for goal in self._goals:
            if goal.start_time + goal.duration > now:
                for publisher in self._cancel_pubs.values():
                    publisher.publish(
                        GoalID(stamp=goal.start_time.to_msg(), id=str(goal))
                    )

    def send_position_hold(self):
        for publisher in self._trajectory_command_pubs.values():
            publisher.publish(JointTrajectory())

    def failed(self) -> bool:
        return self._failed

    def reset(self):
        self._failed = False
        self._goals = []

    def _done_cb(self, result):
        if result.result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
            self._node.get_logger().error(
                f"Failed to execute trajectory. {result.result.error_string} ({result.result.error_code})"
            )
            self._failed = True
            self.cancel_active_goals()

    def get_joint_names_per_controller(self) -> Dict[ControllerName, List[str]]:
        """Get all joint names per controller.

        Returns a dictionary with controller name as key and a list of joint names
        corresponding to that controller as value.

        :return All joint names per controller.
        """
        joint_names_per_controller = {}

        get_param_string_list_client = self._node.create_client(
            srv_type=GetParamStringList,
            srv_name="/march/parameter_server/get_param_string_list",
        )
        for controller_name in self._active_controller_names:
            future = get_param_string_list_client.call_async(
                GetParamStringList.Request(
                    name=f"/march/controller/{controller_name}/joints"
                )
            )
            rclpy.spin_until_future_complete(self._node, future)
            joint_names_per_controller[controller_name] = future.result().value

        return joint_names_per_controller

    @staticmethod
    def extract_joint_trajectory_by_joint_names(
        trajectory: JointTrajectory, joint_names: List[str]
    ) -> JointTrajectory:
        """
        Extract a JointTrajectory with only the points relevant to the supplied joint names.

        :param trajectory: Base joint trajectory to extract points from
        :param joint_names: Which joints to look for
        :return: Returns a JointTrajectory with only the points relevant to the supplied joint names.
        """
        joint_indices = [trajectory.joint_names.index(joint) for joint in joint_names]

        return JointTrajectory(
            header=trajectory.header,
            joint_names=joint_names,
            points=[
                TrajectoryScheduler.copy_joint_trajectory_point_by_indices(
                    point, joint_indices
                )
                for point in trajectory.points
            ],
        )

    @staticmethod
    def copy_joint_trajectory_point_by_indices(
        point: JointTrajectoryPoint, indices: List[int]
    ) -> JointTrajectoryPoint:
        """Copy a JointTrajectoryPoint using a list of indices.

        Repeatedly calls get_values_from_joint_trajectory_list.
        """
        return JointTrajectoryPoint(
            positions=TrajectoryScheduler.get_values_from_joint_trajectory_list(
                point.positions, indices
            ),
            velocities=TrajectoryScheduler.get_values_from_joint_trajectory_list(
                point.velocities, indices
            ),
            accelerations=TrajectoryScheduler.get_values_from_joint_trajectory_list(
                point.accelerations, indices
            ),
            effort=TrajectoryScheduler.get_values_from_joint_trajectory_list(
                point.effort, indices
            ),
            time_from_start=point.time_from_start,
        )

    @staticmethod
    def get_values_from_joint_trajectory_list(
        target_list: list, indices: List[int]
    ) -> list:
        """Get multiple values from a list, using a list of indices."""
        if len(target_list) == 0:
            return []

        return [target_list[index] for index in indices]
