"""This module contains general helper functions.

These functions are not a part of any specific part of the code, but will be useful
for general use cases.
"""
import os
from typing import List, Optional
import math

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from march_utility.exceptions.general_exceptions import SideSpecificationError
from march_utility.utilities.vector_3d import Vector3d
from march_utility.gait.limits import Limits
from .side import Side

import yaml

MARCH_URDF = march_urdf = get_package_share_directory("march_description") + "/urdf/march8.urdf"
MODE_READING = "r"


def weighted_average_floats(base_value: float, other_value: float, parameter: float) -> float:
    """Compute the weighted average of two element with normalised weight parameter.

    Args:
        base_value (float): The first value for the weighted average,
        other_value (float): The second value for the weighted average,
        parameter (float): The normalised weight parameter,
            the parameter that determines the weight of the second value.

    Returns:
        float. The value for the weighted average of the given values.
            - `other_value`: If `parameter` is 0.
            - `base_value`: If `parameter` is 1.
    """
    return base_value * (1 - parameter) + other_value * parameter


def weighted_average_vectors(base_vector: Vector3d, other_vector: Vector3d, parameter: float) -> Vector3d:
    """Compute the weighted average of two element with normalised weight parameter.

    Args:
        base_vector (Vector3d): The first vector for the weighted average.
        other_vector (Vector3d): The second vector for the weighted average.
        parameter (float): The normalised weight parameter, the parameter that
            determines the weight of the second vector.

    Returns:
        Vector3d. A vector which is the weighted average of the given vectors.
            - `other_vector`: If `parameter` is 0.
            - `base_vector`: If `parameter` is 1.
    """
    return base_vector * (1 - parameter) + other_vector * parameter


def merge_dictionaries(dic_one: dict, dic_two: dict) -> dict:
    """Combine the key value pairs of two dictionaries into a new dictionary.

    Throws an error when both dictionaries contain the same key and the
    corresponding values are not equal.

    Args:
        dic_one (dict): First dictionary that is to be merged.
        dic_two (dict): Second dictionary that is to be merged.

    Returns:
        The merged dictionary, has the same key value pairs as the pairs in the given dictionaries combined.
    """
    merged_dic = {}
    for key_one in dic_one:
        if check_key(dic_one, dic_two, key_one):
            merged_dic[key_one] = dic_one[key_one]
    for key_two in dic_two:
        if check_key(dic_one, dic_two, key_two):
            merged_dic[key_two] = dic_two[key_two]

    return merged_dic


def check_key(dic_one: dict, dic_two: dict, key: str) -> bool:
    """Check whether this key can be merged between two dictionaries."""
    if key not in dic_one or key not in dic_two:
        return True
    else:
        if dic_one[key] != dic_two[key]:
            raise KeyError(f"Dictionaries to be merged both contain key {key} with differing values")
        return True


def select_lengths_for_inverse_kinematics(lengths: List[float], side: Side = Side.both) -> List[float]:
    """Return only the lengths in the list on the requested side."""
    if len(lengths) != 9:
        Node("march_utility").get_logger().error("The lengths given did not have size 9. Cannot unpack the lengths.")

    l_ul, l_ll, l_hl, l_ph, r_ul, r_ll, r_hl, r_ph, base = lengths
    if side == Side.left:
        return [l_ul, l_ll, l_hl, l_ph, base]
    elif side == Side.right:
        return [r_ul, r_ll, r_hl, r_ph, base]
    return lengths


def get_lengths_robot_from_urdf_for_inverse_kinematics(  # noqa: CCR001
    length_names: Optional[List[str]] = None,
    side: Side = Side.both,
) -> List[float]:
    """Grab lengths which are relevant for the inverse kinematics calculation from the urdf file.

    This function returns the lengths relevant for the specified foot, if no
    side is specified,it returns all relevant lengths for both feet.

    Args:
        length_names (List[str], Optional): The link length names for which lengths you wish to retrieve.
            Default is `None`, meaning it will get all link lengths.
        side (Side): The side of the exoskeleton of which the lengths would like to be known. Default is `Side.both`.

    Returns:
        List[float]. The lengths of the specified side which are relevant for the (inverse) kinematics calculations
    """
    if not isinstance(side, Side):
        raise SideSpecificationError(side, f"Side should be either 'left', 'right' or 'both', but was {side}")
    try:
        with open(
            os.path.join(
                get_package_share_directory("march_description"),
                "urdf",
                "properties",
                "properties_march8.yaml",
            ),
            MODE_READING,
        ) as yaml_file:
            robot_dimensions = yaml.safe_load(yaml_file)["dimensions"]

        base_length = robot_dimensions["hip_base"]["length"]
        hip_side_length = robot_dimensions["hip_aa_side"]["length"]
        hip_front_length = robot_dimensions["hip_aa_front"]["length"]
        upper_leg_length = robot_dimensions["upper_leg"]["length"]
        lower_leg_length = robot_dimensions["lower_leg"]["length"]
        ankle_offset = robot_dimensions["upper_leg"]["offset"] + robot_dimensions["ankle_plate"]["offset"]
        hip_aa_arm_length = hip_side_length - ankle_offset

    except KeyError as e:
        raise KeyError(f"Expected robot.link_map to contain {e.args[0]}, but it was missing.")

    if length_names is None:
        return select_lengths_for_inverse_kinematics(
            [
                upper_leg_length,
                lower_leg_length,
                hip_front_length,
                hip_aa_arm_length,
                upper_leg_length,
                lower_leg_length,
                hip_front_length,
                hip_aa_arm_length,
                base_length,
            ],
            side,
        )
    else:
        lengths = []
        for name in length_names:
            lengths.append(robot_dimensions[name]["length"])
        return lengths


LENGTHS_BOTH_SIDES = get_lengths_robot_from_urdf_for_inverse_kinematics()


def get_limits_robot_from_urdf_for_inverse_kinematics(joint_name: str):
    """Get the joint from the urdf robot with the given joint name and return the limits of the joint.

    Retrieves it from the `MARCH_URDF`.

    Args:
        joint_name (str): The name to look for.

    Returns:
        float. The limit of the given joint.
    """
    # get limits from march7 yaml
    robot_path = get_package_share_directory("march_hardware_builder") + "/robots/Hennie/march8.yaml"
    with open(robot_path) as file:
        document = yaml.full_load(file)

    robot_name = list(document.keys())[0]
    joints = document[robot_name]["joints"]
    for joint in joints:
        for name in joint:
            if name == joint_name:
                motor_controller = joint[name]["motor_controller"]
                absolute_encoder = motor_controller["absoluteEncoder"]

    total_iu = pow(2, absolute_encoder["resolution"])
    rad_per_iu = (2 * math.pi) / total_iu
    zero_pos = absolute_encoder["zeroPositionIU"]

    lower_soft_error_lim = absolute_encoder["lowerSoftLimitMarginRad"]
    upper_soft_error_lim = absolute_encoder["upperSoftLimitMarginRad"]

    upper_iu = absolute_encoder["maxPositionIU"]
    lower_iu = absolute_encoder["minPositionIU"]
    upper_rad = (upper_iu - zero_pos) * rad_per_iu - upper_soft_error_lim
    lower_rad = (lower_iu - zero_pos) * rad_per_iu + lower_soft_error_lim
    velocity_limit = 3.0
    return Limits(upper=upper_rad, lower=lower_rad, velocity=velocity_limit)


def get_lengths_robot_for_inverse_kinematics(side: Side = Side.both) -> List[float]:
    """Grab lengths which are relevant for the inverse kinematics calculations from a list."""
    return select_lengths_for_inverse_kinematics(LENGTHS_BOTH_SIDES, side)


def validate_and_get_joint_names_for_inverse_kinematics(
    logger=None,
) -> Optional[List[str]]:
    """Get a list of the joint names that can be used for the inverse kinematics.

    Returns:
        None. If the robot description does not contain the required joints.
        List[str]. Otherwise, a list of joint names.
    """
    joint_name_list = [
        "left_ankle",
        "left_hip_aa",
        "left_hip_fe",
        "left_knee",
        "left_ankle",
        "right_hip_aa",
        "right_hip_fe",
        "right_knee",
    ]
    return sorted(joint_name_list)


def get_joint_names_from_urdf(return_fixed_joints: bool = False):
    """Gets a list of all the joint names from the URDF. Filters out the fixed joint if not explicitly asked to return these too.

    Retrieves it from the `MARCH_URDF`.
    """
    robot_path = get_package_share_directory("march_hardware_builder") + "/robots/Hennie/march8.yaml"
    with open(robot_path) as file:
        document = yaml.full_load(file)

    joint_names = []
    robot_name = list(document.keys())[0]
    for joint in document[robot_name]["joints"]:
        for name in joint:
            joint_names.append(name)

    return sorted(joint_names)


def get_position_from_yaml(position: str):
    """Gets a dictionary for default joint angles and given positions.

    Note:
        Gets the position from the 'default.yaml` located in the "march_gait_files" package in "airgait_vi".

    Args:
        position (str): Name of the position, e.g. "stand", for home stand.

    Returns:
        dict[str, float]. The str is the joint name and the float is the joint angle in radians.
    """
    try:
        with open(
            os.path.join(
                get_package_share_directory("march_gait_files"),
                "airgait_vi",
                "default.yaml",
            ),
            MODE_READING,
        ) as yaml_file:
            try:
                return yaml.safe_load(yaml_file)["positions"][position]["joints"]
            except KeyError as e:
                raise KeyError(f"No position found with name {e}")
    except FileNotFoundError as e:
        Node("march_utility").get_logger().error(e)
