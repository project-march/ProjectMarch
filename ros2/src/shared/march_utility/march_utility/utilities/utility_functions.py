"""This module contains general helper functions.

These functions are not a part of any specific part of the code, but will be useful
for general use cases.
"""

import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from urdf_parser_py import urdf
from rclpy.node import Node

from march_utility.exceptions.general_exceptions import SideSpecificationError
from march_utility.utilities.vector_3d import Vector3d
from .side import Side


def weighted_average_floats(
    base_value: float, other_value: float, parameter: float
) -> float:
    """
    Compute the weighted average of two element with normalised weight parameter.

    :param base_value: The first value for the weighted average,
        return this if parameter is 0
    :param other_value: The second value for the weighted average,
        return this if parameter is 1
    :param parameter: The normalised weight parameter, the parameter that
        determines the weight of the second value

    :return: A vector which is the weighted average of the given values
    """
    return base_value * (1 - parameter) + other_value * parameter


def weighted_average_vectors(
    base_vector: Vector3d, other_vector: Vector3d, parameter: float
) -> Vector3d:
    """
    Compute the weighted average of two element with normalised weight parameter.

    :param base_vector: The first vector for the weighted average,
        return this if parameter is 0
    :param other_vector: The second vector for the weighted average,
        return this if parameter is 1
    :param parameter: The normalised weight parameter, the parameter that
        determines the weight of the second vector

    :return: A vector which is the weighted average of the given vectors
    """
    return base_vector * (1 - parameter) + other_vector * parameter


def merge_dictionaries(dic_one: dict, dic_two: dict) -> dict:
    """Combine the key value pairs of two dicitonaries into a new dictionary.

    Throws an error when both dictionaries contain the same key and the
    corresponding values are not equal.
    :param dic_one: One of the dictionaries which is to be merged
    :param dic_two: One of the dictionaries which is to be merged
    :return: The merged dictionary, has the same key value pairs as the pairs in
    the given dictionaries combined
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
            raise KeyError(
                f"Dictionaries to be merged both contain key {key} with differing "
                f"values"
            )
        return True


def select_lengths_for_inverse_kinematics(
    lengths: List[float], side: Side = Side.both
) -> List[float]:
    """Return only the lengths in the list on the requested side."""
    if len(lengths) != 9:
        Node("march_utility").get_logger().error(
            "The lengths given did not have size 9. Cannot unpack the lengths."
        )

    l_ul, l_ll, l_hl, l_ph, r_ul, r_ll, r_hl, r_ph, base = lengths
    if side == Side.left:
        return [l_ul, l_ll, l_hl, l_ph, base]
    elif side == Side.right:
        return [r_ul, r_ll, r_hl, r_ph, base]
    return lengths


def get_lengths_robot_from_urdf_for_inverse_kinematics(  # noqa: CCR001
    side: Side = Side.both,
) -> List[float]:
    """Grab lengths which are relevant for the inverse kinematics calculation from the urdf file.

    This function returns the lengths relevant for the specified foot, if no
    side is specified,it returns all relevant lengths for both feet.

    :param side: The side of the exoskeleton of which the lengths would like to be known
    :return: The lengths of the specified side which are relevant for
        the (inverse) kinematics calculations
    """
    if not isinstance(side, Side):
        raise SideSpecificationError(
            side, f"Side should be either 'left', 'right' or 'both', but was {side}"
        )
    try:
        robot = urdf.Robot.from_xml_file(
            os.path.join(
                get_package_share_directory("march_description"), "urdf", "march4.urdf"
            )
        )
        # size[0], size[1] and size[2] are used to grab relevant length of the link,
        # e.g. the relevant length of the hip base is in the y direction, that of the
        # upper leg in the z direction.
        # length of the hip base structure
        base = robot.link_map["hip_base"].collisions[0].geometry.size[1]  # noqa: ECE001

        # left upper leg length
        l_ul = (  # noqa: ECE001
            robot.link_map["upper_leg_left"].collisions[0].geometry.size[2]
        )

        # left lower leg length
        l_ll = (  # noqa: ECE001
            robot.link_map["lower_leg_left"].collisions[0].geometry.size[2]
        )

        # left haa arm to leg
        l_hl = (  # noqa: ECE001
            robot.link_map["hip_aa_frame_left_front"].collisions[0].geometry.size[0]
        )

        # left pelvis to hip length
        l_ph = (  # noqa ECE001
            robot.link_map["hip_aa_frame_left_side"].collisions[0].geometry.size[1]
        )

        # right upper leg length
        r_ul = (  # noqa: ECE001
            robot.link_map["upper_leg_right"].collisions[0].geometry.size[2]
        )

        # right lower leg length
        r_ll = (  # noqa: ECE001
            robot.link_map["lower_leg_right"].collisions[0].geometry.size[2]
        )

        # right haa arm to leg
        r_hl = (  # noqa: ECE001
            robot.link_map["hip_aa_frame_right_front"].collisions[0].geometry.size[0]
        )

        # right pelvis hip length
        r_ph = (  # noqa: ECE001
            robot.link_map["hip_aa_frame_right_side"]
            .collisions[0]
            .geometry.size[1]  # noqa ECE001
        )
        # The foot is a certain amount more to the inside of the exo then the
        # leg structures. The haa arms (pelic to hip lengths) need to account for this.
        off_set = (  # noqa: ECE001
            robot.link_map["ankle_plate_right"].visual.origin.xyz[1] + base / 2 + r_hl
        )
        r_ph = r_ph - off_set
        l_ph = l_ph - off_set

    except KeyError as e:
        raise KeyError(
            f"Expected robot.link_map to contain {e.args[0]}, but it was missing."
        )

    return select_lengths_for_inverse_kinematics(
        [l_ul, l_ll, l_hl, l_ph, r_ul, r_ll, r_hl, r_ph, base], side
    )


LENGTHS_BOTH_SIDES = get_lengths_robot_from_urdf_for_inverse_kinematics()


def get_lengths_robot_for_inverse_kinematics(side: Side = Side.both) -> List[float]:
    """Grab lengths which are relevant for the inverse kinematics calculations from a list."""
    return select_lengths_for_inverse_kinematics(LENGTHS_BOTH_SIDES, side)


def get_joint_names_for_inverse_kinematics() -> List[str]:
    """Get a list of the joint names that can be used for the inverse kinematics.

    This also checks whether robot description contains the required joints.
    :return: A list of joint names.
    """
    robot = urdf.Robot.from_xml_file(
        os.path.join(
            get_package_share_directory("march_description"), "urdf", "march4.urdf"
        )
    )
    robot_joint_names = robot.joint_map.keys()
    joint_name_list = [
        "left_hip_aa",
        "left_hip_fe",
        "left_knee",
        "right_hip_aa",
        "right_hip_fe",
        "right_knee",
    ]
    for joint_name in joint_name_list:
        if joint_name not in robot_joint_names:
            raise KeyError(
                f"Inverse kinematics calculation expected the robot to have joint "
                f"{joint_name}, but {joint_name} was not found."
            )

    return joint_name_list
