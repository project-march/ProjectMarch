cmake_minimum_required(VERSION 2.8.3)
project(march_rqt_gait_generator)

find_package(catkin REQUIRED COMPONENTS
        rviz
        roslint
        march_shared_resources
        )

catkin_python_setup()

catkin_package(
        CATKIN_DEPENDS
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
