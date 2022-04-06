"""Author: Marten Haitjema, MVII."""


import os
import yaml
import numpy as np
from ament_index_python import get_package_share_path

from march_goniometric_ik_solver.ik_solver import Pose
from march_utility.gait.setpoint import Setpoint
from march_utility.utilities.duration import Duration
from march_utility.utilities.utility_functions import get_position_from_yaml, get_joint_names_from_urdf
from march_gait_selection.dynamic_interpolation.dynamic_joint_trajectory import DynamicJointTrajectory
from march_gait_selection.dynamic_interpolation.dynamic_subgait import INTERPOLATION_POINTS


def main():
    pose = Pose()

    # Constant variables
    middle_point_fraction = 0.45
    middle_point_height = 0.15
    name = "perform_test"
    gait_type = ""

    # Variable input
    file_name = input("File name: ")  # "perform_test_dynamic_v0"
    description = input("Description: ")
    joint_name = input("Joint to use: ")
    duration = float(input("Duration (in seconds): "))
    x = float(input("x location: "))
    y = float(input("y location: "))
    z = float(input("z location (0.45 for no sidestep): "))

    # Get positions in lists
    start_position = get_position_from_yaml("stand")
    middle_position = pose.solve_mid_position(x, y, z, middle_point_fraction, middle_point_height, "right_swing")
    end_position = pose.solve_end_position(x, y, z, "right_swing")

    # Get setpoint
    joint_names = get_joint_names_from_urdf()
    index = joint_names.index(joint_name)
    start_position_setpoint = Setpoint(Duration(0), start_position[joint_name], 0)
    middle_position_setpoint = Setpoint(Duration(middle_point_fraction*duration), middle_position[index], 0)
    end_position_setpoint = Setpoint(Duration(duration), end_position[index], 0)

    setpoint_list = [start_position_setpoint, middle_position_setpoint, end_position_setpoint]
    if joint_name in ["right_ankle", "left_ankle"]:
        trajectory = DynamicJointTrajectory(setpoint_list, interpolate_ankle=True)
    else:
        trajectory = DynamicJointTrajectory(setpoint_list)

    # Get trajectory points
    interpolation_points = np.linspace(0, duration, INTERPOLATION_POINTS)
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
        "description": description,
        "duration": duration,
        "gait_type": gait_type,
        "joints": {"rotational_joint": list_dict_of_setpoints},
        "name": name,
    }

    # Write to yaml file
    path = get_package_share_path("march_gait_files")
    file_name += ".subgait"
    if joint_name in ["left_knee", "left_hip_fe", "right_knee", "right_hip_fe"]:
        directory = os.path.join(
            path, "test_joint_rotational_gaits", "test_joint_gait", "perform_test", file_name,
        )
    else:
        directory = os.path.join(
            path, "test_joint_linear_gaits", "test_joint_gait", "perform_test", file_name,
        )

    with open(directory, "w") as subgait_file:
        yaml.dump(dictionary, subgait_file)


if __name__ == "__main__":
    main()
