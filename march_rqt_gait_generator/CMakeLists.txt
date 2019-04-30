cmake_minimum_required(VERSION 2.8.3)
project(march_rqt_gait_generator)

find_package(catkin REQUIRED COMPONENTS
        rviz
        roslint
        message_generation
        message_runtime
        std_msgs
        trajectory_msgs

        )

catkin_python_setup()

add_message_files(
        FILES
        ActualSetpoint.msg
        MarchGait.msg
        Sound.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        trajectory_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_generation
        message_runtime
        std_msgs
)

install(FILES plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(DIRECTORY resource
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(PROGRAMS scripts/rqt_gait_generator
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

file(GLOB_RECURSE lintfiles
        "src/*.py"
        )

roslint_python(${lintfiles})
