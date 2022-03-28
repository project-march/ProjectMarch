from copy import deepcopy
from threading import Event

from march_gait_selection.state_machine.gait_update import GaitUpdate
from march_gait_selection.state_machine.gait_interface import GaitInterface
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand
from march_shared_msgs.srv import CapturePointPose, GetMoveItTrajectory
from march_utility.gait.gait import Gait
from march_utility.utilities.duration import Duration
from march_utility.utilities.logger import Logger
from rclpy import Future
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory


class BalanceGait(GaitInterface):
    """Base class to create a gait using the moveit motion planning.

    Args:
        node (Node): node that subscribers will be made on
        gait_name (:obj: str, optional): name of the gait, default is 'balanced_walk'
        default_walk (:obj: Gait, optional): default is None

    Attributes:
        gait_name (str): name of the gait
        logger (Logger): used to log to the terminal
        capture_point_event (Event): ???
        capture_point_result (???): ???

        _node (Node): node used to create subscribers
        _default_walk (:obj: Gait, optional): default is None
        _constructing (bool): ???
        _current_subgait (Subgait): current subgait
        _current_subgait_duration (Duration): duration of current subgait
        _start_time (rclpy.Time): time at which subgait should start
        _end_time (rclpy.Time): time at which subgait should end
        _current_time (rclpy.Time): current time
    """

    CAPTURE_POINT_SERVICE_TIMEOUT = 1.0
    MOVEIT_INTERFACE_SERVICE_TIMEOUT = 1.0

    def __init__(
        self, node: Node, gait_name: str = "balanced_walk", default_walk: Gait = None
    ):
        self.gait_name = gait_name
        self._node = node
        self._default_walk = default_walk
        self._constructing = False
        self.logger = Logger(self._node, __class__.__name__)

        self._current_subgait = None
        self._current_subgait_duration = Duration(0)

        self._start_time = None
        self._end_time = None
        self._current_time = None

        self.capture_point_event = Event()
        self.capture_point_result = None
        self._capture_point_service = {
            "left_leg": node.create_client(
                srv_name="/march/capture_point/foot_left", srv_type=CapturePointPose
            ),
            "right_leg": node.create_client(
                srv_name="/march/capture_point/foot_right", srv_type=CapturePointPose
            ),
        }

        self.moveit_event = Event()
        self.moveit_trajectory_result = None
        self._moveit_trajectory_service = node.create_client(
            srv_name="/march/moveit/get_trajectory", srv_type=GetMoveItTrajectory
        )

    @property
    def default_walk(self) -> Gait:
        """Return the default walk subgait."""
        return self._default_walk

    @default_walk.setter
    def default_walk(self, new_default_walk: Gait):
        """Set a new default walk subgait to the balance subgait.

        Args:
            new_default_walk (Gait): A new subgait which is the default balance walking pattern
        """
        self._default_walk = new_default_walk

    def compute_swing_leg_target(self, leg_name: str, subgait_name: str) -> bool:
        """Set the swing leg target to capture point.

        Args:
            leg_name (str): name of the used move group
            subgait_name (str): the normal subgait name
        """
        subgait_duration = self.default_walk[subgait_name].duration
        if not self._capture_point_service[leg_name].wait_for_service(timeout_sec=3):
            self.logger.warn(
                f"Capture point service not found: "
                f"{self._capture_point_service[leg_name]}"
            )

        self.capture_point_event.clear()

        future = self._capture_point_service[leg_name].call_async(
            CapturePointPose.Request(duration=subgait_duration.seconds)
        )
        future.add_done_callback(self.capture_point_cb)
        return self.capture_point_event.wait(timeout=self.CAPTURE_POINT_SERVICE_TIMEOUT)

    def capture_point_cb(self, future: Future) -> None:
        """Set capture point result when the capture point service returns."""
        self.capture_point_result = future.result()
        self.capture_point_event.set()

    def compute_stance_leg_target(self, leg_name: str, subgait_name: str) -> JointState:
        """Set the target of the stance leg to the end of the gait file.

        Args:
            leg_name (str): name of the used move group
            subgait_name (str): the normal subgait name
        Returns:
            JointState: message containing name, position and velocity
        """
        side_prefix = "right" if "right" in leg_name else "left"

        default_subgait = deepcopy(self.default_walk[subgait_name])

        non_capture_point_joints = []
        for joint in default_subgait.joints:
            if side_prefix in joint.name:
                non_capture_point_joints.append(joint)

        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self._node.get_clock().now().to_msg()
        joint_state.name = [joint.name for joint in non_capture_point_joints]
        joint_state.position = [
            joint.setpoints[-1].position for joint in non_capture_point_joints
        ]
        joint_state.velocity = [
            joint.setpoints[-1].velocity for joint in non_capture_point_joints
        ]

        return joint_state

    def construct_trajectory(
        self, swing_leg: str, subgait_name: str
    ) -> JointTrajectory:
        """Constructs a balance trajectory for all joints.

        Args:
            swing_leg (str): name of the used move group
            subgait_name (str): the normal subgait name
        Returns:
            JointTrajectory: the balance trajectory
        """
        if swing_leg not in ["right_leg", "left_leg"]:
            self.logger.warn(
                f"Swing leg was not one of the possible legs "
                f"(left_leg or right_leg), but {swing_leg}, "
                f"using default walk instead"
            )
            return self.default_walk[subgait_name].to_joint_trajectory_msg()
        stance_leg = "right_leg" if swing_leg == "left_leg" else "left_leg"
        capture_point_success = self.compute_swing_leg_target(swing_leg, subgait_name)
        if not capture_point_success:
            self.logger.warn("Capture point call took too long, using default gait.")
            return self.default_walk[subgait_name].to_joint_trajectory_msg()
        stance_leg_target = self.compute_stance_leg_target(stance_leg, subgait_name)

        self.moveit_event.clear()

        request = GetMoveItTrajectory.Request(
            swing_leg=swing_leg,
            swing_leg_target_pose=self.capture_point_result.capture_point,
            stance_leg_target=stance_leg_target,
        )
        trajectory_future = self._moveit_trajectory_service.call_async(request)

        trajectory_future.add_done_callback(self.moveit_event_cb)
        if self.moveit_event.wait(self.MOVEIT_INTERFACE_SERVICE_TIMEOUT):
            return self.moveit_trajectory_result.trajectory
        else:
            self.logger.warn("Moveit interface call took too long, using default gait.")
            return self.default_walk[subgait_name].to_joint_trajectory_msg()

    def moveit_event_cb(self, future: Future):
        """Set moveit trajectory result when the capture point service returns."""
        self.moveit_trajectory_result = future.result()
        self.moveit_event.set()

    def get_joint_trajectory_msg(self, name: str) -> JointTrajectory:
        """Returns the trajectory of a subgait name that could use moveit.

        Args:
            name (str): name of the subgait
        Returns
            JointTrajectory: message containing trajectory
        """
        if name in ["right_open_2", "right_swing_2"]:  # noqa: SIM116
            return self.construct_trajectory("right_leg", name)
        elif name == "left_swing_2":
            return self.construct_trajectory("left_leg", name)
        else:
            return self.default_walk[name].to_joint_trajectory_msg()

    # GaitInterface
    @property
    def name(self):
        return self.gait_name

    @property
    def subgait_name(self):
        return self._current_subgait

    @property
    def duration(self):
        return self._current_subgait_duration

    @property
    def gait_type(self):
        return "walk_like"

    @property
    def starting_position(self):
        return self._default_walk.starting_position

    @property
    def final_position(self):
        return self._default_walk.final_position

    def start(self, current_time: Time) -> GaitUpdate:
        self._current_time = current_time
        self._current_subgait = self._default_walk.graph.start_subgaits()[0]
        return GaitUpdate.should_schedule(self._new_trajectory_command())

    def update(self, current_time: Time) -> GaitUpdate:
        self._current_time = current_time
        if self._current_time < self._end_time or self._constructing:
            return GaitUpdate.empty()
        else:
            next_subgait = self._default_walk.graph[
                (self._current_subgait, self._default_walk.graph.TO)
            ]

            if next_subgait == self._default_walk.graph.END:
                return GaitUpdate.finished()
            self._constructing = True
            self._current_subgait = next_subgait
            command = self._new_trajectory_command()
            self._constructing = False
            return GaitUpdate.should_schedule(command)

    def _new_trajectory_command(self) -> TrajectoryCommand:
        """Update the trajectory values and generate a new trajectory command.

        Returns:
            TrajectoryCommand: a TrajectoryCommand for the next subgait.
        """
        trajectory = self.get_joint_trajectory_msg(self._current_subgait)
        time_from_start = trajectory.points[-1].time_from_start
        self._current_subgait_duration = Duration.from_msg(time_from_start)
        self._start_time = self._current_time
        self._end_time = self._start_time + self._current_subgait_duration
        return TrajectoryCommand(
            trajectory,
            self._current_subgait_duration,
            self.subgait_name,
            self._start_time,
        )

    def end(self) -> None:
        self._current_subgait = None
        self._current_subgait_duration = Duration(0)
