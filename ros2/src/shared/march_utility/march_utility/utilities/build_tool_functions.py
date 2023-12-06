"""This module contains helper functions to use in your package 'setup.py'.

Note:
     It is important to add `<buildtool_depend>march_utility</buildtool_depend>` to your package.xml.
     After that you can import the functions in your `setup.py` with:
        rom march_utility.utilities.utility_functions import ...
"""
import os
from glob import glob
from typing import List, Tuple

from launch.substitutions import PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare


def copy_subdir(dir_path: str, file_extension: str, package_name: str) -> List[Tuple]:
    """This method is made to load in subdirectories with their files extensions.

    The data_files are required in the format of tuples (install_path, source_path).
    With install_path referencing to the generated folder in 'install/[package_name]',
    and the source_path referencing to the path the files are currently (pre_build) located.

    Important:
        **This method doesn't work for more than 1 level deep.**

    Example:
        ```
        data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*.launch.py"))),
        ]`

        **data_files.extend(copy_subdir("config", '*', package_name))**
        ```

    How it works:
        If this method is called with `copy_subdir('config', '*.yaml')`,
        it will load in all files located in '`pwd`/config/[folders]/[file_names].yaml'
        into '[project_dir]/install/[package_name]/share/[package_name]/config/[folders]/[file_names].yaml'

    Args:
        dir_path (str): The directory from package level you want to copy over, e.g. 'config'.
        file_extension (str): The types of files it needs to copy over, e.g. '*.yaml'.
            You can also do '*' to copy over all types of files.
        package_name (str): The name of the package, e.g. 'march_utility'.

    Returns:
        List[Tuple[str, List[str]]]. A list of tuple of the format (build_folder_name, [files_copied_to_this_folder]).
    """
    ret_list = []
    for name in glob(os.path.join(dir_path, "*")):
        subdir = name.split(os.sep)[-1]
        ret_list.append(
            (
                os.path.join("share", package_name, dir_path, subdir),
                glob(os.path.join(dir_path, subdir, file_extension)),
            )
        )
    return ret_list


# def generate_robot_desc_command(
#     robot_descr_file, ground_gait, realsense_simulation, simulation, jointless, gazebo_control_yaml
# ):
def generate_robot_desc_command(
    robot_descr_file, ground_gait, simulation, jointless
):
    """Generates the robot description command to load in the xacro file with arguments.

    Args:
        robot_descr_file (str): The xacro to use. Must be located in `.../march_description/urdf/... .xacro`.
        ground_gait (bool): Whether we want to have the exo on the ground or fixed to the world.
        realsense_simulation (bool): Whether we want to use realsense in the simulation.
        simulation (bool): True if we run it in rviz or gazebo.
        jointless (bool): True if we run it without any joints at all.
        gazebo_control_yaml (str): The file where the control parameters are located when running with gazebo.
                                    Must be in `.../march_control/config/[gazebo_control_yaml]`

    Returns:
        str. The command that can be run to parse the xacro.
    """
    return [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("march_description"), "urdf", robot_descr_file]),
        ".xacro",
        " ground_gait:=",
        ground_gait,
        " realsense_simulation:=",
        # realsense_simulation,
        # " configuration:=",
        # ("exoskeleton" if not simulation else "simulation"),
        # " gazebo_control:=",
        simulation,
        " jointless:=",
        jointless,
        # " gazebo_control_file:=",
    # get_control_file_loc(gazebo_control_yaml),
    ]


def get_control_file_loc(control_yaml):
    """To get the absolute path the control file.

    Args:
        control_yaml (str): The name of the control file looked for (Must contain file extension, e.g. '.yaml').
                             Must be in `.../march_control/config/[gazebo_control_yaml]`

    Returns:
        str. The  absolute path the control file
    """
    return PathJoinSubstitution([FindPackageShare("march_control"), "config", control_yaml])
