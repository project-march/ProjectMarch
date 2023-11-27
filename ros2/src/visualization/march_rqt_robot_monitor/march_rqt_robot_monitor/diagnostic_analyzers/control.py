"""The module control.py contains the CheckJointValues Class."""
import contextlib
from typing import Optional, List

from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.node import Node

from rclpy.time import Time
from sensor_msgs.msg import JointState

WARN_PERCENTAGE = 5


class CheckJointValues:
    """Base class to diagnose the joint movement values."""

    def __init__(self, node: Node, joints, topic: str, msg_type: type):
        node.create_subscription(msg_type, topic, self.cb, qos_profile=10)
        self.node = node
        self.joints = joints

        # callback variables
        self._timestamp: Optional[Time] = None
        self._joint_names: Optional[List[str]] = None
        self._position: Optional[List[float]] = None
        self._velocity: Optional[List[float]] = None
        self._effort: Optional[List[float]] = None

        # robot properties
        self._lower_soft_limits = {}
        self._upper_soft_limits = {}

        for joint in self.joints:
            self.set_joint_limits(joint)

    def set_joint_limits(self, joint):
        """Set the joint limits for a joint.

        :param joint  Joint dict to get limits from.
        """
        with contextlib.suppress(AttributeError):
            for name in joint.keys():
                self._lower_soft_limits[name] = joint[name]["motor_controller"]["absoluteEncoder"]["minPositionIU"]
                self._upper_soft_limits[name] = joint[name]["motor_controller"]["absoluteEncoder"]["minPositionIU"]

    def cb(self, msg: JointState):
        """Save the latest published movement values with corresponding timestamp."""
        self._timestamp = Time.from_msg(msg.header.stamp)
        self._joint_names = msg.name
        self._position = msg.position
        self._velocity = msg.velocity
        self._effort = msg.effort

    def position_diagnostics(self, stat):
        """The diagnostic message to display the positions in standard format."""
        if self._timestamp is None:
            stat.add("Topic error", "No events recorded")
            stat.summary(DiagnosticStatus.STALE, "No position recorded")
            return stat

        stat.add("Timestamp", str(self._timestamp))

        joint_outside_soft_limits = []
        joint_in_warning_zone_soft_limits = []

        for index in range(len(self._joint_names)):
            joint_name = self._joint_names[index]
            stat.add(self._joint_names[index], str(self._position[index]))

            if self._position[index] >= self._upper_soft_limits[joint_name]:
                joint_outside_soft_limits.append(self._joint_names[index])
            elif self._position[index] >= (self._upper_soft_limits[joint_name] * (1 - WARN_PERCENTAGE / 100)):
                joint_in_warning_zone_soft_limits.append(self._joint_names[index])

            if self._position[index] <= self._lower_soft_limits[joint_name]:
                joint_outside_soft_limits.append(self._joint_names[index])
            elif self._position[index] <= (self._lower_soft_limits[joint_name] * (1 - WARN_PERCENTAGE / 100)):
                joint_in_warning_zone_soft_limits.append(self._joint_names[index])

        if joint_outside_soft_limits:
            stat.summary(
                DiagnosticStatus.ERROR,
                f"Outside soft limits: {joint_outside_soft_limits}",
            )
        elif joint_in_warning_zone_soft_limits:
            stat.summary(
                DiagnosticStatus.WARN,
                f"Close to soft limits: {joint_in_warning_zone_soft_limits}",
            )
        else:
            stat.summary(DiagnosticStatus.OK, "OK")

        return stat

    def velocity_diagnostics(self, stat):
        """The diagnostic message to display the velocities in standard format."""
        if self._timestamp is None:
            stat.add("Topic error", "No events recorded")
            stat.summary(DiagnosticStatus.STALE, "No velocity recorded")
            return stat

        stat.add("Timestamp", str(self._timestamp))

        joints_at_velocity_limit = []
        joint_in_warning_zone_velocity_limit = []

        for index in range(len(self._joint_names)):
            joint_name = self._joint_names[index]
            stat.add(self._joint_names[index], str(self._velocity[index]))

            if self._velocity[index] >= self._velocity_limits[joint_name]:
                joints_at_velocity_limit.append(self._joint_names[index])
            elif self._velocity[index] >= (self._velocity_limits[joint_name] * (1 - WARN_PERCENTAGE / 100)):
                joint_in_warning_zone_velocity_limit.append(self._joint_names[index])

        if joints_at_velocity_limit:
            stat.summary(
                DiagnosticStatus.ERROR,
                "At velocity limit: {ls}".format(ls=str(joints_at_velocity_limit)),
            )
        elif joint_in_warning_zone_velocity_limit:
            stat.summary(
                DiagnosticStatus.WARN,
                "Close to velocity limit: {ls}".format(ls=str(joint_in_warning_zone_velocity_limit)),
            )
        else:
            stat.summary(DiagnosticStatus.OK, "OK")

        return stat

    def effort_diagnostics(self, stat):
        """The diagnostic message to display the efforts in standard format."""
        if self._timestamp is None:
            stat.add("Topic error", "No events recorded")
            stat.summary(DiagnosticStatus.STALE, "No effort recorded")
            return stat

        stat.add("Timestamp", str(self._timestamp))

        joints_at_effort_limit = []
        joints_in_warning_zone_effort_limits = []

        for index in range(len(self._joint_names)):
            joint_name = self._joint_names[index]
            stat.add(self._joint_names[index], str(self._position[index]))

            if self._effort[index] >= self._effort_limits[joint_name]:
                joints_at_effort_limit.append(self._joint_names[index])
            elif self._effort[index] >= (self._effort_limits[joint_name] * (1 - WARN_PERCENTAGE / 100)):
                joints_in_warning_zone_effort_limits.append(self._joint_names[index])

        if joints_at_effort_limit:
            stat.summary(
                DiagnosticStatus.ERROR,
                "At effort limit: {ls}".format(ls=str(joints_at_effort_limit)),
            )
        elif joints_in_warning_zone_effort_limits:
            stat.summary(
                DiagnosticStatus.WARN,
                "Close to effort limit: {ls}".format(ls=str(joints_in_warning_zone_effort_limits)),
            )
        else:
            stat.summary(DiagnosticStatus.OK, "OK")

        return stat
