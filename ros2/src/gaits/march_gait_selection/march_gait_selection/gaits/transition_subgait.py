"""Author: ???."""

from copy import deepcopy
from typing import List

from march_utility.exceptions.gait_exceptions import TransitionError
from march_utility.gait.joint_trajectory import JointTrajectory
from march_utility.gait.setpoint import Setpoint
from march_utility.gait.subgait import Subgait
from march_utility.utilities.duration import Duration


class TransitionSubgait(Subgait):
    """Class that defines the subgait used for transitioning between subgaits with not matching begin and end points.

    Args:
        joints (List[JointTrajectory]): list containing joint trajectories for each joint
        duration (Duration): duration of the subgait
        gait_type (:obj: str, optional): Type of the gait, defaults to 'walk_like'
        gait_name (:obj: str, optional): Name of the gait, defaults to 'Walk'
        subgait_name (:obj: str, optional): Name of the subgait, defaults to 'right_open'
        version (:obj: str, optional): Version of the subgait, defaults to 'First try'
        description (:obj: str, optional): Description of the subgait, defaults to 'Just a simple gait'
    """

    def __init__(
        self,
        joints: List[JointTrajectory],
        duration: Duration,
        gait_type="walk_like",
        gait_name="Transition",
        subgait_name="Transition_subgait",
        version="Default",
        description="The subgait used to transition between two subgaits",
    ):
        super(TransitionSubgait, self).__init__(
            joints, duration, gait_type, gait_name, subgait_name, version, description
        )

    @classmethod
    def from_subgaits(cls, old_subgait: Subgait, new_subgait: Subgait, transition_subgait_name: str):
        """Create a new transition subgait object between two given subgaits.

        Args:
            old_subgait (Subgait): the old subgait to transition from
            new_subgait (Subgait): the new gait which must be executed after the old gait
            transition_subgait_name (str): Name to use for the subgait that will be created in which the
                transition will occur
        Returns:
            TransitionSubgait: A populated TransitionSubgait object which holds the data to transition
                between given gaits
        """
        old_subgait_copy = deepcopy(old_subgait)
        new_subgait_copy = deepcopy(new_subgait)
        transition_joints = cls._transition_joints(old_subgait_copy, new_subgait_copy)
        transition_duration = new_subgait_copy.duration

        transition_subgait = cls(transition_joints, transition_duration, subgait_name=transition_subgait_name)

        cls._validate_transition_gait(old_subgait_copy, transition_subgait, new_subgait_copy)
        cls._validate_transition_trajectory(old_subgait_copy, transition_subgait, new_subgait_copy)

        return transition_subgait

    @staticmethod
    def _transition_joints(old_subgait: Subgait, new_subgait: Subgait) -> List[JointTrajectory]:
        """Calculate a transition trajectory which starts at the old gait and ends with the endpoints of the new gait.

        Args:
            old_subgait (Subgait): the old subgait to transition from
            new_subgait (Subgait): the new gait which must be executed after the old gait
        Returns:
            List[JointTrajectory]: list of joints which hold the transition setpoints including
                 position, velocity and duration
        """
        max_duration = max(old_subgait.duration, new_subgait.duration)

        old_subgait.scale_timestamps_subgait(max_duration)
        new_subgait.scale_timestamps_subgait(max_duration)

        all_timestamps = old_subgait.get_unique_timestamps() + new_subgait.get_unique_timestamps()
        all_timestamps = sorted(set(all_timestamps))

        old_subgait.create_interpolated_setpoints(all_timestamps)
        new_subgait.create_interpolated_setpoints(all_timestamps)

        joints = []
        for old_joint in old_subgait.joints:

            joint_name = old_joint.name
            new_joint = new_subgait.get_joint(joint_name)

            setpoints = []
            number_setpoints = len(new_subgait[0].setpoints)
            for transition_index in range(number_setpoints):
                factor = transition_index / (number_setpoints - 1.0)

                old_setpoint = old_joint[transition_index]
                new_setpoint = new_joint[transition_index]

                transition_setpoint = TransitionSubgait._transition_setpoint(old_setpoint, new_setpoint, factor)
                setpoints.append(transition_setpoint)

            joints.append(JointTrajectory(joint_name, old_joint.limits, setpoints, old_joint.duration))

        return joints

    @staticmethod
    def _transition_setpoint(old_setpoint: Setpoint, new_setpoint: Setpoint, new_factor: float) -> Setpoint:
        """Create a transition setpoint with the use of the old setpoint, new setpoint and transition factor.

        Args:
            old_setpoint (Setpoint): old setpoint to transition from
            new_setpoint (Setpoint): new setpoint to transition to
            new_factor (float): weight factor between old and new setpoints
        Returns:
            Setpoint: setpoint with scaled position and velocity and time of new_setpoint
        """
        old_factor = 1.0 - new_factor

        position = (old_setpoint.position * old_factor) + (new_setpoint.position * new_factor)
        velocity = (old_setpoint.velocity * old_factor) + (new_setpoint.velocity * new_factor)

        return Setpoint(new_setpoint.time, position, velocity)

    @staticmethod
    def _validate_transition_gait(old_subgait: Subgait, transition_subgait: Subgait, new_subgait: Subgait) -> None:
        """Validates the transition point.

        Args:
            old_subgait (Subgait): old subgait to validate transition
            transition_subgait (Subgait): transitions to validate
            new_subgait (Subgait): new subgait to validate transition
        Raises:
            TransitionError: raised when transition is not valid
        """
        try:
            if not old_subgait.validate_subgait_transition(
                transition_subgait
            ) or not transition_subgait.validate_subgait_transition(new_subgait):
                raise TransitionError("Transition subgaits do not match")
        except Exception as error:  # noqa: PIE786
            TransitionError("Error when creating transition: {er}".format(er=error))

    @staticmethod
    def _validate_transition_trajectory(
        old_subgait: Subgait, transition_subgait: Subgait, new_subgait: Subgait
    ) -> None:
        """Validate if the calculated trajectory is within the given subgaits.

        Args:
            old_subgait (Subgait): old subgait to validate transition
            transition_subgait (Subgait): transitions to validate
            new_subgait (Subgait): new subgait to validate transition
        Raises:
            TransitionError: raised when transition is not valid
        """
        for transition_joint in transition_subgait.joints:
            old_joint = old_subgait.get_joint(transition_joint.name)
            new_joint = new_subgait.get_joint(transition_joint.name)

            for old_setpoint, transition_setpoint, new_setpoint in zip(old_joint, transition_joint, new_joint):

                if old_setpoint.time != transition_setpoint.time:
                    raise TransitionError(
                        "The transition timestamp {tt} != the old timestamp {ot}".format(
                            tt=transition_setpoint.time, ot=old_setpoint.time
                        )
                    )

                if new_setpoint.time != transition_setpoint.time:
                    raise TransitionError(
                        "The transition timestamp {tt} != the new timestamp {ot}".format(
                            tt=transition_setpoint.time, ot=new_setpoint.time
                        )
                    )

                if (
                    old_setpoint.position < transition_setpoint.position
                    and new_setpoint.position < transition_setpoint.position
                ):
                    raise TransitionError(
                        "The transition position {tp} is not between the "
                        "old {op} and new {np}".format(
                            tp=transition_setpoint.position,
                            op=old_setpoint.position,
                            np=new_setpoint.position,
                        )
                    )

                if (
                    old_setpoint.position > transition_setpoint.position
                    and new_setpoint.position > transition_setpoint.position
                ):
                    raise TransitionError(
                        "The transition position {tp} is not between the "
                        "new {np} and old {op}".format(
                            tp=transition_setpoint.position,
                            op=old_setpoint.position,
                            np=new_setpoint.position,
                        )
                    )
