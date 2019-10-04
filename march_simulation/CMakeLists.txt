cmake_minimum_required(VERSION 2.8.3)
project(march_simulation)

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    gazebo_plugins
    gazebo_ros
    gazebo_ros_control
    march_description
    robot_state_publisher
    xacro
)

catkin_package()

catkin_install_python(PROGRAMS scripts/upload_joint_names
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch worlds
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
