from copy import deepcopy
import os
import sys
from threading import Event
from typing import Optional

from rclpy import Future
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from march_gait_selection.state_machine.gait_interface import GaitInterface
from march_shared_msgs.srv import CapturePointPose, GetMoveItTrajectory


class BalanceGait(GaitInterface):
    """Base class to create a gait using the moveit motion planning."""

    CAPTURE_POINT_SERVICE_TIMEOUT = 1
    NANOSEC_TO_SEC = 10e-9

    def __init__(self, node, gait_name="balanced_walk", default_walk=None):
        self.gait_name = gait_name
        self._node = node
        self._default_walk = default_walk

        self.moveit_event = Event()

        self._capture_point_service = {
            "left_leg": node.create_client(
                srv_name="/march/capture_point/foot_left", srv_type=CapturePointPose
            ),
            "right_leg": node.create_client(
                srv_name="/march/capture_point/foot_right", srv_type=CapturePointPose
            ),
        }

        self._moveit_trajectory_service = node.create_client(
            srv_name="/march/moveit/get_trajectory", srv_type=GetMoveItTrajectory
        )

        self._current_subgait = None
        self._current_subgait_duration = 0.0
        self._time_since_start = 0.0

    @property
    def default_walk(self):
        """Return the default walk subgait."""
        return self._default_walk

    @default_walk.setter
    def default_walk(self, new_default_walk):
        """Set a new default walk subgait to the balance subgait.

        :param new_default_walk:
            A new subgait which is the default balance walking pattern
        """
        self._default_walk = new_default_walk

    def compute_swing_leg_target(self, leg_name: str, subgait_name: str) -> \
            Optional[JointState]:
        """Set the swing leg target to capture point.

        :param leg_name: The name of the used move group
        :param subgait_name: the normal subgait name
        """
        self._node.get_logger().info('Computing swing leg target')
        subgait_duration = self.default_walk[subgait_name].duration
        return_msg = self._capture_point_service[leg_name](
            duration=subgait_duration)

        if not return_msg.success:
            self._node.get_logger().warn("No messages received from the capture "
                                           "point service.")
            return None

        self._node.get_logger().info(
            f"Done comuputing swing leg target, duration is {return_msg.duration}")
        return return_msg.duration

    def compute_stance_leg_target(self, leg_name: str, subgait_name: str) -> JointState:
        """Set the target of the stance leg to the end of the gait file.

        :param leg_name: The name of the move group which does not use capture point
        :param subgait_name: the normal subgait name

        :return the duration of the default subgait
        """
        side_prefix = "right" if "right" in leg_name else "left"

        default_subgait = deepcopy(self.default_walk[subgait_name])

        non_capture_point_joints = []
        for joint in default_subgait.joints:
            if side_prefix in joint.name:
                non_capture_point_joints.append(joint)

        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self._node.get_clock().now()
        joint_state.name = [joint.name for joint in non_capture_point_joints]
        joint_state.position = [
            joint.setpoints[-1].position for joint in non_capture_point_joints
        ]
        joint_state.velocity = [
            joint.setpoints[-1].velocity for joint in non_capture_point_joints
        ]

        return joint_state

    def construct_trajectory(self, swing_leg, subgait_name):
        """Constructs a balance trajectory for all joints.

        :param capture_point_leg_name: The name of the move group that should be used to create the balance subgait
        :param subgait_name: the normal subgait name

        :return: the balance trajectory
        """
        stance_leg = "right_leg" if swing_leg == "left_leg" else "left_leg"
        self._node.get_logger().info('Constructing trajectory')    
        swing_leg_target = self.compute_swing_leg_target(swing_leg, subgait_name)
        stance_leg_target = self.compute_stance_leg_target(stance_leg, subgait_name)

        self.moveit_event.clear()
        self._node.get_logger().info('Time for moveit interface call') 
        trajectory_future = \
            self._moveit_trajectory_service.call_async(GetMoveItTrajectory.Request(
                swing_leg=swing_leg, swing_leg_target=swing_leg_target,
                stance_leg_target=stance_leg_target))

        trajectory_future.add_done_callback(self.moveit_event_cb)
        self.moveit_event.wait()

        return self.moveit_trajectory_result.trajectory if self.moveit_trajectory_result.success else \
            self.default_walk[
            subgait_name].to_joint_trajectory_msg()

    def moveit_event_cb(self, future: Future):
        result = future.result()
        self.moveit_trajectory_result = result
        self.moveit_event.set()

    def get_joint_trajectory_msg(self, name):
        """Returns the trajectory of a subgait name that could use moveit.

        :param name: the name of the subgait
        """
        self._node.get_logger().info(f'Time to get_joint_trajectory_msg {name}')
        if name == "right_open_2":
            self._node.get_logger().info('RIGHT OPEN 2')
            return self.construct_trajectory("right_leg", name)
        elif name == "left_swing_2":
            return self.construct_trajectory("left_leg", name)
        elif name == "right_swing_2":
            return self.construct_trajectory("right_leg", name)
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

    def start(self):
        self._current_subgait = self._default_walk.graph.start_subgaits()[0]
        self._time_since_start = 0.0
        trajectory = self.get_joint_trajectory_msg(self._current_subgait)
        time_from_start = trajectory.points[-1].time_from_start
        self._current_subgait_duration = float(time_from_start.sec)  + \
                                         (time_from_start.nanosec *
                                         self.NANOSEC_TO_SEC)
        return trajectory

    def update(self, elapsed_time):
        self._time_since_start += elapsed_time
        if self._time_since_start < self._current_subgait_duration:
            return None, False
        else:
            next_subgait = self._default_walk.graph[
                (self._current_subgait, self._default_walk.graph.TO)
            ]

            if next_subgait == self._default_walk.graph.END:
                return None, True

            self._node.get_logger().info(next_subgait)

            trajectory = self.get_joint_trajectory_msg(next_subgait)
            self._current_subgait = next_subgait
            self._current_subgait_duration = trajectory.points[
                -1
            ].time_from_start.to_sec()
            self._time_since_start = 0.0
            return trajectory, False

    def end(self):
        self._current_subgait = None
        self._current_subgait_duration = 0.0