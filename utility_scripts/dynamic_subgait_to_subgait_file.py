"""Author: Marten Haitjema, MVII."""

import os
import yaml
import argparse
import numpy as np
from ament_index_python import get_package_share_path

from march_goniometric_ik_solver.ik_solver import Pose
from march_utility.gait.setpoint import Setpoint
from march_utility.utilities.duration import Duration
from march_utility.utilities.utility_functions import get_joint_names_from_urdf
from march_gait_selection.dynamic_interpolation.dynamic_joint_trajectory import DynamicJointTrajectory
from march_gait_selection.dynamic_interpolation.dynamic_subgait import INTERPOLATION_POINTS


def main():
    """Script that writes a dynamic joint trajectory to a .subgait file to be used with the test setup.

    Note:
        Make sure you have sourced ros2 and foxy.

    Example:
        Add the following alias to run the script:

        `alias create_subgait_script='sfox && sros2 &&
        python3 ~/march/utility_scripts/dynamic_subgait_to_subgait_file.py'`
    """
    joint_names_from_urdf = get_joint_names_from_urdf()
    pose = Pose()

    help_text = """
Script that writes a dynamic joint trajectory to a .subgait file to be used with the test setup.
Files are written to the install folder: ros2/install/march_gait_files/share/march_gait_files/... Always creates
a 'right_swing' subgait.
    """
    parser = argparse.ArgumentParser(description=help_text, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument(
        "file_name",
        type=str,
        default="perform_test_dynamic_v0",
        help="Name given to the .subgait file, for example 'perform_test_dynamic_v0'",
    )
    parser.add_argument(
        "description",
        type=str,
        help="Description that will be set in the .subgait file",
    )
    parser.add_argument("joint_name", type=str, help="Joint to use", choices=joint_names_from_urdf)
    parser.add_argument(
        "duration",
        type=float,
        help="Duration of the subgait in seconds.",
    )
    parser.add_argument(
        "x",
        type=float,
        help="Step size. Usually between 0.1 and 0.5",
    )
    parser.add_argument(
        "y",
        type=float,
        help="Step height. Usually between -0.2 and 0.2",
    )
    parser.add_argument(
        "z",
        type=float,
        help="Side step. Use 0.45 for no sidestep",
    )
    parser.add_argument(
        "--middle_point_fraction",
        type=float,
        help="Fraction of step at which middle point will be set",
        default=0.45,
    )
    parser.add_argument(
        "--middle_point_height",
        type=float,
        help="Height that middle point will be above end point",
        default=0.15,
    )
    parser.add_argument(
        "--name",
        type=str,
        help="Name of the gait. Should be perform_test for the button on the ipd to work",
        default="perform_test",
    )
    parser.add_argument(
        "--gait_type",
        type=str,
        help="Type of the gait, for example 'stairs_like' or 'walk_like'",
        default="",
    )
    args = parser.parse_args()

    # Get positions in lists
    middle_position = pose.solve_mid_position(
        args.x, args.y, args.z, args.middle_point_fraction, args.middle_point_height, "right_swing"
    )
    end_position = pose.solve_end_position(args.x, args.y, args.z, "right_swing")
    # start position is mirrored end position
    end_position_left_leg = end_position[0:4]
    start_position = end_position[4:8]
    for pos in end_position_left_leg:
        start_position.append(pos)

    # Get setpoint
    index = joint_names_from_urdf.index(args.joint_name)
    start_position_setpoint = Setpoint(Duration(0), start_position[index], 0)
    middle_position_setpoint = Setpoint(Duration(args.middle_point_fraction * args.duration), middle_position[index], 0)
    end_position_setpoint = Setpoint(Duration(args.duration), end_position[index], 0)

    setpoint_list = [start_position_setpoint, middle_position_setpoint, end_position_setpoint]
    if args.joint_name in ["right_ankle", "left_ankle"]:
        trajectory = DynamicJointTrajectory(setpoint_list, interpolate_ankle=True)
    else:
        trajectory = DynamicJointTrajectory(setpoint_list)

    # Get trajectory points
    interpolation_points = np.linspace(0, args.duration, INTERPOLATION_POINTS)
    trajectory_list = []
    for point in interpolation_points:
        trajectory_list.append(trajectory.get_interpolated_setpoint(point))

    # Create dict
    list_dict_of_setpoints = []
    for setpoint in trajectory_list:
        list_dict_of_setpoints.append(
            {"position": setpoint.position, "time_from_start": setpoint.time.seconds, "velocity": setpoint.velocity}
        )

    dictionary = {
        "description": args.description,
        "duration": args.duration,
        "gait_type": args.gait_type,
        "joints": {},
        "name": args.name,
    }

    # Write to yaml file
    path = get_package_share_path("march_gait_files")
    args.file_name += ".subgait"
    if args.joint_name in ["left_knee", "left_hip_fe", "right_knee", "right_hip_fe"]:
        dictionary["joints"] = {"rotational_joint": list_dict_of_setpoints}
        joint_type_path = "test_joint_rotational_gaits"
    else:
        dictionary["joints"] = {"linear_joint": list_dict_of_setpoints}
        joint_type_path = "test_joint_linear_gaits"

    file_path = os.path.join(path, joint_type_path, "test_joint_gait", "perform_test", args.file_name)

    with open(file_path, "w") as subgait_file:
        yaml.dump(dictionary, subgait_file)

    print(f"Successfully created {args.file_name} file.")


if __name__ == "__main__":
    main()
