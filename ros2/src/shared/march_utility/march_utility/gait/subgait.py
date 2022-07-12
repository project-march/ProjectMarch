"""Defines a base class for subgaits that can be executed by the exoskeleton."""
from __future__ import annotations

import os
import re
from typing import List, Tuple, Collection, Dict

import yaml
from march_utility.exceptions.gait_exceptions import (
    NonValidGaitContentError,
    SubgaitInterpolationError,
    GaitError,
)
from march_utility.foot_classes.feet_state import FeetState
from march_utility.utilities.duration import Duration
from march_utility.utilities.utility_functions import (
    validate_and_get_joint_names_for_inverse_kinematics,
)
from march_utility.utilities.dimensions import InterpolationDimensions
from trajectory_msgs import msg as trajectory_msg
from urdf_parser_py import urdf

from .joint_trajectory import JointTrajectory
from .limits import Limits
from .setpoint import Setpoint
from ..exceptions.gait_exceptions import UnknownDimensionsError
from ..utilities.dimensions import amount_of_subgaits, amount_of_parameters

PARAMETRIC_GAITS_PREFIX = "_pg_"
FOUR_PARAMETRIC_GAITS_PREFIX = "_fpg_"
SUBGAIT_SUFFIX = ".subgait"
JOINT_NAMES_IK = validate_and_get_joint_names_for_inverse_kinematics()


class Subgait:
    """Base class for usage of the defined subgaits.

    Args:
        joints (List[JointTrajectory]): list containing joint trajectories for each joint
        duration (Duration): duration of the subgait
        gait_type (:obj: str, optional): Type of the gait, defaults to 'walk_like'
        gait_name (:obj: str, optional): Name of the gait, defaults to 'Walk'
        subgait_name (:obj: str, optional): Name of the subgait, defaults to 'right_open'
        version (:obj: str, optional): Version of the subgait, defaults to 'First try'
        description (:obj: str, optional): Description of the subgait, defaults to 'Just a simple gait'
        robot (:obj: urdf.Robot, optional): robot model to use, default is None
    Attributes:
        joints (List[JointTrajectory]): list containing joint trajectories for each joint
        duration (Duration): duration of the subgait
        gait_type (:obj: str, optional): Type of the gait, defaults to 'walk_like'
        gait_name (:obj: str, optional): Name of the gait, defaults to 'Walk'
        subgait_name (:obj: str, optional): Name of the subgait, defaults to 'right_open'
        version (:obj: str, optional): Version of the subgait, defaults to 'First try'
        description (:obj: str, optional): Description of the subgait, defaults to 'Just a simple gait'
        robot (:obj: urdf.Robot, optional): robot model to use, default is None
    """

    joint_class = JointTrajectory

    def __init__(
        self,
        joints: List[JointTrajectory],
        duration: Duration,
        gait_type: str = "walk_like",
        gait_name: str = "Walk",
        subgait_name: str = "right_open",
        version: str = "First try",
        description: str = "Just a simple gait",
        robot: urdf.Robot = None,
    ) -> None:
        self.joints = joints
        self.gait_type = gait_type
        self.gait_name = gait_name
        self.robot = robot
        self.subgait_name = subgait_name
        self.version = version
        self.description = str(description)
        self.duration = duration

    # region Create subgait
    @classmethod
    def from_file(cls, robot: urdf.Robot, file_name: str) -> Subgait:
        """Extract sub gait data of the given yaml.

        Args:
            robot (urdf.Robot): robot model to use
            file_name (str): name of the file
        Returns:
            Subgait: A populated Subgait object
        Raises:
            TypeError: if robot or file_name is not specified
        """
        if robot is None:
            raise TypeError("Robot is None, should be a valid urdf.Robot object")
        if file_name is None:
            raise TypeError("Filename is None, should be a string")

        with open(file_name, "r") as yaml_file:
            subgait_dict = yaml.load(yaml_file, Loader=yaml.SafeLoader)

        gait_name = file_name.split("/")[-3]
        subgait_name = file_name.split("/")[-2]
        version = file_name.split("/")[-1].replace(SUBGAIT_SUFFIX, "")

        return cls.from_dict(robot, subgait_dict, gait_name, subgait_name, version)

    @classmethod
    def from_name_and_version(
        cls,
        robot: urdf.Robot,
        gait_dir: str,
        gait_name: str,
        subgait_name: str,
        version: str,
    ) -> Subgait:
        """Load subgait based from file(s) based on name and version.

        Args:
            robot (urdf.Robot): The robot corresponding to the given subgait file
            gait_dir (str): The directory with all the gaits
            gait_name (str): The name of the corresponding gait
            subgait_name (str): The name of the subgait to load
            version (str): The version to use, this can be parametric
        Returns:
            Subgait: A populated Subgait object
        """
        subgait_path = os.path.join(gait_dir, gait_name, subgait_name)
        if version.startswith(PARAMETRIC_GAITS_PREFIX):
            base_version, other_version, parameter = Subgait.unpack_parametric_version(version)
            base_version_path = os.path.join(subgait_path, base_version + SUBGAIT_SUFFIX)
            if base_version == other_version:
                return cls.from_file(robot, base_version_path)
            else:
                other_version_path = os.path.join(subgait_path, other_version + SUBGAIT_SUFFIX)
                return cls.from_two_files_interpolated(
                    robot,
                    base_version_path,
                    other_version_path,
                    parameter,
                    use_foot_position=True,
                )
        elif version.startswith(FOUR_PARAMETRIC_GAITS_PREFIX):
            version_path_list = [""] * 4
            (
                gait_version_list,
                parameter_list,
            ) = Subgait.unpack_four_parametric_version(version)
            for version_index in range(4):
                version_path_list[version_index] = os.path.join(
                    subgait_path, gait_version_list[version_index] + SUBGAIT_SUFFIX
                )
            return cls.from_four_files_interpolated(
                robot,
                version_path_list,
                parameter_list,
                use_foot_position=True,
            )

        else:
            subgait_version_path = os.path.join(subgait_path, version + SUBGAIT_SUFFIX)
            return cls.from_file(robot, subgait_version_path)

    @classmethod
    def from_four_files_interpolated(
        cls,
        robot: urdf.Robot,
        version_path_list: List[str, str, str, str],
        parameter_list: List[float, float],
        use_foot_position: bool = False,
    ) -> Subgait:
        """Extract two subgaits from files and interpolate.

        Args:
            robot (urdf.robot): The robot corresponding to the given subgait file
            version_path_list (List[str]): The .yaml file names of the subgaits to interpolate
            parameter_list (List[float]): The parameters to use for interpolation. Should all be between 0 and 1
            use_foot_position (:obj: bool, optional): Determine whether the interpolation should be done on the
                foot location or on the joint angles
        Returns:
            Subgait: A populated Subgait object
        """
        subgaits = []
        for i in range(4):
            subgaits.append(cls.from_file(robot, version_path_list[i]))
        return cls.interpolate_four_subgaits(
            subgaits,
            parameter_list,
            use_foot_position,
        )

    @classmethod
    def from_two_files_interpolated(
        cls,
        robot: urdf.Robot,
        first_file_name: str,
        second_file_name: str,
        first_parameter: float,
        use_foot_position: bool = False,
    ) -> Subgait:
        """Extract two subgaits from files and interpolate.

         Args:
            robot (urdf.robot): The robot corresponding to the given subgait file.
            first_file_name (str): The .yaml file name of the base subgait.
            second_file_name (str): The .yaml file name of the subgait.
            first_parameter (float): The parameter to use for interpolation. Should be 0 <= parameter <= 1.
            use_foot_position (:obj: bool, optional): Determine whether the interpolation should be done on the
                foot location or on the joint angles.

        Returns:
            Subgait: A populated Subgait object.
        """
        base_subgait = cls.from_file(robot, first_file_name)
        other_subgait = cls.from_file(robot, second_file_name)
        return cls.interpolate_subgaits(base_subgait, other_subgait, first_parameter, use_foot_position)

    @classmethod
    def from_dict(
        cls,
        robot: urdf.Robot,
        subgait_dict: dict,
        gait_name: str,
        subgait_name: str,
        version: str,
    ) -> Subgait:
        """List parameters from the yaml file in organized lists.

        Args:
            robot (urdf.Robot): The robot corresponding to the given subgait file
            subgait_dict (dict): The dictionary extracted from the yaml file
            gait_name (str): Name of the parent gait
            subgait_name (str): Name of the child (sub)gait
            version (str): The version of the yaml file
        Returns:
            Subgait: A populated Subgait object
        """
        if robot is None:
            raise GaitError("Cannot create gait without a loaded robot.")

        duration = Duration(nanoseconds=subgait_dict["duration"])
        joint_list = []
        for name, points in sorted(subgait_dict["joints"].items(), key=lambda item: item[0]):
            urdf_joint = cls.joint_class.get_joint_from_urdf(robot, name)
            if urdf_joint is None or urdf_joint.type == "fixed":
                continue
            limits = Limits.from_urdf_joint(urdf_joint)
            joint_list.append(cls.joint_class.from_setpoint_dict(name, limits, points, duration))
        subgait_type = subgait_dict["gait_type"] if subgait_dict.get("gait_type") else ""
        subgait_description = subgait_dict["description"] if subgait_dict.get("description") else ""

        return cls(
            joint_list,
            duration,
            subgait_type,
            gait_name,
            subgait_name,
            version,
            subgait_description,
            robot,
        )

    # endregion

    # region Create messages
    def to_joint_trajectory_msg(self) -> trajectory_msg.JointTrajectory:
        """Create trajectory msg for the publisher.

        Returns:
            JointTrajectory: a ROS msg for the joint trajectory
        """
        joint_trajectory_msg = trajectory_msg.JointTrajectory()

        joint_trajectory_msg.joint_names = [joint.name for joint in self.joints]

        timestamps = self.get_unique_timestamps()
        for timestamp in timestamps:
            joint_trajectory_point = trajectory_msg.JointTrajectoryPoint()
            joint_trajectory_point.time_from_start = timestamp.to_msg()

            for joint in self.joints:
                interpolated_setpoint = joint.get_interpolated_setpoint(timestamp)

                joint_trajectory_point.positions.append(interpolated_setpoint.position)
                joint_trajectory_point.velocities.append(interpolated_setpoint.velocity)

            joint_trajectory_msg.points.append(joint_trajectory_point)

        return joint_trajectory_msg

    # endregion

    # region Validate subgait
    def validate_subgait_transition(self, next_subgait: Subgait) -> bool:
        """Validate the trajectory transition of this gait to a given gait.

        Args:
            next_subgait (Subgait): The subgait subsequently to this gait (not the previous one!).

        Returns:
            bool: `True` if trajectory transition correct else `False`.
        """
        from_subgait_joint_names = set(self.get_joint_names())
        to_subgait_joint_names = set(next_subgait.get_joint_names())

        if from_subgait_joint_names != to_subgait_joint_names:
            raise NonValidGaitContentError(
                msg="Gait {gait}, structure of joints does not match between "
                "subgait {fn} and subgait {tn}".format(
                    gait=self.gait_name,
                    fn=self.subgait_name,
                    tn=next_subgait.subgait_name,
                )
            )

        for joint_name in to_subgait_joint_names:
            from_joint = self.get_joint(joint_name)
            to_joint = next_subgait.get_joint(joint_name)
            if not from_joint.validate_joint_transition(to_joint):
                return False

        return True

    # endregion

    # region Manipulate subgait
    def scale_timestamps_subgait(self, new_duration: Duration, rescale: bool = True) -> None:
        """Scale or cut off all the setpoint to match the duration in both subgaits.

        Args:
            new_duration (Duration): the new duration to scale the setpoints with
            rescale (bool): set to true if all points should be rescaled, alternative is cut off after new duration.

        """
        new_duration = round(new_duration, Setpoint.digits)

        for joint in self.joints:
            joint.set_duration(new_duration, rescale)
        self.duration = new_duration

    def create_interpolated_setpoints(self, timestamps: List[Duration]) -> None:
        """Equalize the setpoints of the subgait match the given timestamps.

        Args:
            timestamps (List[Duration]): the new timestamps to use when creating the setpoints
        """
        timestamps = sorted(set(timestamps + self.get_unique_timestamps()))

        for joint in self.joints:
            new_joint_setpoints = []
            for timestamp in timestamps:
                if timestamp > self.duration:
                    raise IndexError(
                        f"Gait {self.gait_name}, subgait {self.subgait_name} could "
                        f"not extrapolate timestamp outside max duration"
                    )

                new_joint_setpoints.append(joint.get_interpolated_setpoint(timestamp))

            joint.setpoints = new_joint_setpoints

    @classmethod
    def interpolate_n_subgaits(
        cls,
        dimensions: InterpolationDimensions,
        subgaits: List[Subgait],
        parameters: List[float],
        use_foot_position: bool,
    ) -> Subgait:
        """Interpolates between N amount of subgaits.

        Args:
            dimensions (InterpolationDimensions): Dimension of interpolation
            subgaits (List[Subgait]): subgaits with which to interpolate, amount should be equal to dimension
            parameters (List[float]): parameters to perform interpolation with, amount should be equal to dimension
            use_foot_position (bool): whether to use foot position or not
        Returns:
            Subgait: A populated Subgait object
        Raises:
            SubgaitInterpolationError: raised when dimensions, subgaits and parameters do not match
            UnknownDimensionsError: raised when dimensions is not equal to InterpolationDimensions.ONE_DIM or
                InterpolationDimensions.TWO_DIM
        """
        if len(subgaits) != amount_of_subgaits(dimensions):
            raise SubgaitInterpolationError("The length of the subgait list does not match the given dimensions")
        if len(parameters) != amount_of_parameters(dimensions):
            raise SubgaitInterpolationError(
                f"The amount of parameters does not match {len(parameters)} the given dimensions {dimensions}"
            )
        if dimensions == InterpolationDimensions.ONE_DIM:
            return cls.interpolate_subgaits(subgaits[0], subgaits[1], parameters[0], use_foot_position)
        elif dimensions == InterpolationDimensions.TWO_DIM:
            return cls.interpolate_four_subgaits(subgaits, parameters, use_foot_position)
        else:
            raise UnknownDimensionsError(dimensions)

    @classmethod
    def interpolate_four_subgaits(
        cls,
        subgaits: List[Subgait, Subgait, Subgait, Subgait],
        parameters: List[float],
        use_foot_position: bool = False,
    ) -> Subgait:
        """Interpolate two subgaits with the parameter to get a new subgait.

        Args:
            subgaits (List[Subgait]): list of subgaits with which to perform interpolation, length should be four
            parameters (float): list of parameters to perform interpolation with. First param is used for interpolation
                between subgait 1&2, second for 3&4. Should be 0 <= parameter <= 1
            use_foot_position (bool): Determine whether the interpolation should be done on the foot
                location or on the joint angles
        Returns:
            Subgait: The interpolated subgait
        """
        first_interpolated_subgait = Subgait.interpolate_subgaits(
            subgaits[0], subgaits[1], parameters[0], use_foot_position
        )
        second_interpolated_subgait = Subgait.interpolate_subgaits(
            subgaits[2], subgaits[3], parameters[0], use_foot_position
        )

        return Subgait.interpolate_subgaits(
            first_interpolated_subgait,
            second_interpolated_subgait,
            parameters[1],
            use_foot_position,
        )

    @classmethod
    def interpolate_subgaits(
        cls,
        base_subgait: Subgait,
        other_subgait: Subgait,
        parameter: float,
        use_foot_position: bool = False,
    ) -> Subgait:
        """Interpolate two subgaits with the parameter to get a new subgait.

        Args:
            base_subgait (Subgait): base subgait, return value if parameter is equal to zero
            other_subgait (Subgait): other subgait, return value if parameter is equal to one
            parameter (float): The parameter to use for interpolation. Should be 0 <= parameter <= 1
            use_foot_position (float): Determine whether the interpolation should be done on the foot
            location or on the joint angles
        Returns:
            Subgait: The interpolated subgait
        Raises:
            ValueError: when parameter is not 0 <= parameter <= 1
        """
        if parameter == 1:
            return other_subgait
        if parameter == 0:
            return base_subgait
        if not (0 < parameter < 1):
            raise ValueError(f"Parameter for interpolation should be in the interval [0, 1], but is {parameter}")

        if use_foot_position:
            joints = Subgait.get_foot_position_interpolated_joint_trajectories(base_subgait, other_subgait, parameter)
        else:
            joints = Subgait.get_joint_angle_interpolated_joint_trajectories(base_subgait, other_subgait, parameter)

        description = (
            f"Interpolation between base version {base_subgait.version}, "
            f"and other version {other_subgait.version} with parameter {parameter}. "
            f"Based on foot position: {use_foot_position}"
        )

        duration = base_subgait.duration.weighted_average(other_subgait.duration, parameter)

        gait_type = base_subgait.gait_type if parameter <= 0.5 else other_subgait.gait_type
        # What to do with the version of the new gait
        version = "{0}{1}_({2})_({3})".format(
            PARAMETRIC_GAITS_PREFIX,
            parameter,
            base_subgait.version,
            other_subgait.version,
        )
        return Subgait(
            joints,
            duration,
            gait_type,
            base_subgait.gait_name,
            base_subgait.subgait_name,
            version,
            description,
        )

    # endregion

    # region Get functions
    def get_unique_timestamps(self, sorted_timestamps: bool = True) -> Collection[Duration]:
        """Get the timestamps that are unique to a setpoint.

        Args:
            sorted_timestamps (bool): True if timestamps should be sorted in increasing order
        Returns:
            Collection[Duration]: the unique timestamps of the subgaits
        """
        timestamps = []
        for joint in self.joints:
            for setpoint in joint.setpoints:
                timestamps.append(setpoint.time)
        if sorted_timestamps:
            return sorted(set(timestamps))
        else:
            return set(timestamps)

    def get_joint(self, name: str) -> JointTrajectory:
        """Get joint object with given name or index.

        Args:
            name (str): name of the joint
        Returns:
            JointTrajectory: JointTrajectory for the given name
        """
        return next(joint for joint in self.joints if joint.name == name)

    def get_joint_names(self) -> List[str]:
        """Get the names of all the joints existing in the joint list.

        Returns:
            List[str]: list containing the joint names
        """
        return [joint.name for joint in self.joints]

    @property
    def starting_position(self) -> Dict[str, float]:
        """Get a dictionary of joint positions at the start of this subgait.

        Returns:
            Dict[str, float]: dict containing joint name and corresponding position
        """
        return {joint.name: joint.setpoints[0].position for joint in self.joints}

    @property
    def final_position(self) -> Dict[str, float]:
        """Get a dictionary of joint positions at the end of this subgait.

        Returns:
            Dict[str, float]: dict containing joint name and corresponding position
        """
        return {joint.name: joint.setpoints[-1].position for joint in self.joints}

    # endregion

    def to_dict(self) -> dict:
        """Get the subgait represented as a dictionary.

        Returns:
            Dict[str, Union[str, float, Dict[str, float]]]: dictionary containing all information from the yaml file
        """
        return {
            "description": self.description,
            "duration": self.duration.nanoseconds,
            "gait_type": self.gait_type,
            "joints": dict(
                {
                    joint.name: [
                        {
                            "position": setpoint.position,
                            "time_from_start": setpoint.time.nanoseconds,
                            "velocity": setpoint.velocity,
                        }
                        for setpoint in joint.setpoints
                    ]
                    for joint in self.joints
                }
            ),
            "name": self.subgait_name,
            "version": self.version,
        }

    def to_yaml(self) -> str:
        """Return a YAML string representation of the subgait.

        Returns:
            str: a yaml string representation of the subgait
        """
        return yaml.dump(self.to_dict())

    # region Class methods
    def __getitem__(self, index):
        """Return joint corresponding to given index."""
        return self.joints[index]

    def __len__(self):
        """Return the length of the list containing joint names."""
        return len(self.joints)

    # endregion

    @staticmethod
    def validate_version(gait_path: str, subgait_name: str, version: str) -> bool:
        """Check whether a gait exists for the gait.

        Args:
            gait_path (str): The path to the gait
            subgait_name (str): The name of the subgait
            version (str): The version of the subgait
        Returns:
             bool: Whether the gait is valid
        """
        subgait_path = os.path.join(gait_path, subgait_name)
        if version.startswith(PARAMETRIC_GAITS_PREFIX):
            Subgait.validate_parametric_version(subgait_path, version)
        elif version.startswith(FOUR_PARAMETRIC_GAITS_PREFIX):
            Subgait.validate_four_parametric_version(subgait_path, version)
        else:
            version_path = os.path.join(subgait_path, version + SUBGAIT_SUFFIX)
            if not os.path.isfile(version_path):
                return False
        return True

    @staticmethod
    def validate_four_parametric_version(subgait_path: str, version: str) -> bool:
        """Check whether a parametric gait is valid.

        Args:
            subgait_path (str): The path to the subgait file
            version (str): The version of the parametric gait
        Returns
            bool: True if the subgait is parametrized between two existing subgaits
        """
        (
            gait_version_list,
            parameter_list,
        ) = Subgait.unpack_four_parametric_version(version)
        version_path_list = [""] * 4
        for version_index in range(4):
            version_path_list[version_index] = os.path.join(
                subgait_path, gait_version_list[version_index] + SUBGAIT_SUFFIX
            )
        return all(not os.path.isfile(version_path) for version_path in version_path_list)

    @staticmethod
    def validate_parametric_version(subgait_path: str, version: str) -> bool:
        """Check whether a parametric gait is valid.

        Args:
            subgait_path (str): The path to the subgait file
            version (str): The version of the parametric gait
        Returns:
            bool: True if the subgait is parametrized between two existing subgaits
        """
        base_version, other_version, _ = Subgait.unpack_parametric_version(version)
        base_version_path = os.path.join(subgait_path, base_version + SUBGAIT_SUFFIX)
        other_version_path = os.path.join(subgait_path, other_version + SUBGAIT_SUFFIX)
        return all(not os.path.isfile(version_path) for version_path in [base_version_path, other_version_path])

    @staticmethod
    def unpack_parametric_version(version: str) -> Tuple[str, str, float]:
        """Unpack a version to base version, other version and parameter.

        Args:
            version (str): version of the subgait to unpack
        Returns:
            Tuple[str, str, float]: the base version, other version and the parameter
        """
        parameter_search = re.search(r"^{0}(\d+\.\d+)_".format(PARAMETRIC_GAITS_PREFIX), version)
        if parameter_search is None:
            raise SubgaitInterpolationError(
                f"Parametric version string was stored in wrong format. "
                f"Version string is {version}. Correct example is "
                f"'_{PARAMETRIC_GAITS_PREFIX}0.5_(version_1)_(version_2)"
            )
        parameter = float(parameter_search.group(1))
        versions = re.findall(r"\([^\)]*\)", version)
        base_version = versions[0][1:-1]
        other_version = versions[1][1:-1]
        return base_version, other_version, parameter

    @staticmethod
    def unpack_four_parametric_version(version: str) -> Tuple[List[str], List[float]]:
        """Unpack a version to four versions and two parameters.

        Args:
            version (str): version of the subgait to unpack
        Returns:
            Tuple[str, str, float]: the base version, other version and the parameter
        Raises:
            SubgaitInterpolationError: raised when version string if stored in wrong format
        """
        parameter_search = re.findall(r"(\d+\.\d+)", version)
        if parameter_search is None:
            raise SubgaitInterpolationError(
                f"Parametric version string was stored in wrong format. "
                f"Version string is {version}. Correct example is "
                f"'_{FOUR_PARAMETRIC_GAITS_PREFIX}_0.5_0.5_(version_1)_(version_2)_(version_3)_(version_4)"
            )
        parameter_list = [float(parameter_search[0]), float(parameter_search[1])]
        versions = re.findall(r"\([^\)]*\)", version)

        version_list = []
        for version in versions:
            version_list.append(version[1:-1])

        return version_list, parameter_list

    @staticmethod
    def check_foot_position_interpolation_is_safe(base_subgait: Subgait, other_subgait: Subgait) -> None:
        """Check whether two subgaits are safe to be interpolated on foot location.

        Args:
            base_subgait (Subgait): the base subgait that will be used for interpolation
            other_subgait (Subgait): the other subgait that will be used for interpolation
        """
        for base_joint, other_joint in zip(
            sorted(base_subgait.joints, key=lambda joint: joint.name),
            sorted(other_subgait.joints, key=lambda joint: joint.name),
        ):
            JointTrajectory.check_joint_interpolation_is_safe(base_joint, other_joint)

    @staticmethod
    def get_foot_position_interpolated_joint_trajectories(
        base_subgait: Subgait, other_subgait: Subgait, parameter: float
    ) -> List[JointTrajectory]:
        """Create a list of joint trajectories by interpolating foot locations.

        The foot location corresponding to the resulting trajectories is equal
        to the weighted average (with the  parameter) of the foot locations
        corresponding to the base and other subgait.

        Args:
            base_subgait (Subgait): return value if parameter is equal to zero
            other_subgait (Subgait): return value if parameter is equal to one
            parameter (float): Parameter for interpolation, between 0 and 1
        Returns:
            List[JointTrajectory]: A list of interpolated joint trajectories
        """
        if JOINT_NAMES_IK is None:
            return base_subgait.joints
        Subgait.check_foot_position_interpolation_is_safe(base_subgait, other_subgait)

        # The inverse kinematics needs access to the 'ith' setpoints of all joints
        (
            base_setpoints_to_interpolate,
            other_setpoints_to_interpolate,
        ) = Subgait.prepare_subgaits_for_inverse_kinematics(base_subgait, other_subgait)

        number_of_setpoints = len(base_setpoints_to_interpolate)

        new_setpoints: dict = {joint.name: [] for joint in base_subgait.joints}
        # fill all joints in new_setpoints except the ankle joints using
        # the inverse kinematics
        for setpoint_index in range(number_of_setpoints):
            if base_setpoints_to_interpolate[setpoint_index] == other_setpoints_to_interpolate[setpoint_index]:
                setpoints_to_add = base_setpoints_to_interpolate[setpoint_index]
            else:
                base_feet_state = FeetState.from_setpoint_dict(base_setpoints_to_interpolate[setpoint_index])
                other_feet_state = FeetState.from_setpoint_dict(other_setpoints_to_interpolate[setpoint_index])
                new_feet_state = FeetState.weighted_average_states(base_feet_state, other_feet_state, parameter)
                setpoints_to_add = FeetState.feet_state_to_setpoints(new_feet_state)

            for joint_name in JOINT_NAMES_IK:
                new_setpoints[joint_name].append(setpoints_to_add[joint_name])
            # fill the ankle joint using the angle based linear interpolation
            for ankle_joint in ["left_ankle", "right_ankle"]:
                base_setpoint = base_setpoints_to_interpolate[setpoint_index][ankle_joint]
                other_setpoint = other_setpoints_to_interpolate[setpoint_index][ankle_joint]
                new_ankle_setpoint_to_add = Setpoint.interpolate_setpoints(base_setpoint, other_setpoint, parameter)
                new_setpoints[ankle_joint].append(new_ankle_setpoint_to_add)

        duration = base_subgait.duration.weighted_average(other_subgait.duration, parameter)

        interpolated_joint_trajectories = [None] * len(base_subgait.joints)
        for index, joint in enumerate(base_subgait.joints):
            interpolated_joint_trajectory_to_add = JointTrajectory(
                joint.name, joint.limits, new_setpoints[joint.name], duration
            )
            interpolated_joint_trajectories[index] = interpolated_joint_trajectory_to_add

        return interpolated_joint_trajectories

    @staticmethod
    def prepare_subgaits_for_inverse_kinematics(
        base_subgait: Subgait, other_subgait: Subgait
    ) -> Tuple[List[dict], List[dict]]:
        """Create two lists of setpoints with equal time stamps.

        Args:
            base_subgait (Subgait): return value if parameter is equal to zero
            other_subgait (Subgait): return value if parameter is equal to one
        Returns:
            Tuple[List[dict], List[dict]]: two lists of setpoint with equal timestamps
        """
        base_to_other_duration_ratio = other_subgait.duration / base_subgait.duration

        original_base_time_stamps = set(base_subgait.get_unique_timestamps(sorted_timestamps=False))
        other_time_stamps = set(other_subgait.get_unique_timestamps(sorted_timestamps=False))

        for base_time in original_base_time_stamps:
            other_time_stamps.add(round((base_time * base_to_other_duration_ratio), Setpoint.digits))

        base_time_stamps = [
            round(other_time / base_to_other_duration_ratio, Setpoint.digits) for other_time in other_time_stamps
        ]

        base_time_stamps = sorted(base_time_stamps)
        other_time_stamps = sorted(other_time_stamps)

        base_setpoints_to_interpolate = Subgait.prepare_subgait_for_inverse_kinematics(base_subgait, base_time_stamps)
        other_setpoints_to_interpolate = Subgait.prepare_subgait_for_inverse_kinematics(
            other_subgait, other_time_stamps
        )

        return base_setpoints_to_interpolate, other_setpoints_to_interpolate

    @staticmethod
    def prepare_subgait_for_inverse_kinematics(subgait: Subgait, time_stamps: List[Duration]) -> List[dict]:
        """Create a list of setpoints from a subgait with timestamps given by time_stamps.

        Args:
            subgait (Subgait): Subgait to prepare for inverse kinematics.
            time_stamps (List[Duration]): Time stamps at which setpoints in the list are set.

        Returns:
            List[dict]: List of setpoints with timestamps given by time_stamps.
        """
        setpoints_to_interpolate: List[dict] = [{} for _ in time_stamps]

        for setpoint_index, time_stamp in enumerate(time_stamps):
            for joint in subgait.joints:
                setpoint_to_add = joint.get_interpolated_setpoint(time_stamp)
                setpoints_to_interpolate[setpoint_index][joint.name] = setpoint_to_add

        return setpoints_to_interpolate

    @staticmethod
    def get_joint_angle_interpolated_joint_trajectories(
        base_subgait: Subgait, other_subgait: Subgait, parameter: float
    ) -> List[JointTrajectory]:
        """Interpolate joint trajectories for each joint trajectory in two subgaits.

        Args:
            base_subgait (Subgait): return value if parameter is equal to zero
            other_subgait (Subgait): return value if parameter is equal to one
            parameter (float): Parameter for interpolation, between 0 and 1
        Returns:
         List[JointTrajectory]: A list of linearly interpolated joint trajectories
        """
        interpolated_joint_trajectories = []
        for base_joint, other_joint in zip(
            sorted(base_subgait.joints, key=lambda joint: joint.name),
            sorted(other_subgait.joints, key=lambda joint: joint.name),
        ):
            if base_joint.name != other_joint.name:
                raise SubgaitInterpolationError(
                    "The subgaits to interpolate do not have the same joints, base"
                    " subgait has {0}, while other subgait has {1}".format(base_joint.name, other_joint.name)
                )
            interpolated_joint_trajectory_to_add = JointTrajectory.interpolate_joint_trajectories(
                base_joint, other_joint, parameter
            )
            interpolated_joint_trajectories.append(interpolated_joint_trajectory_to_add)

        return interpolated_joint_trajectories
