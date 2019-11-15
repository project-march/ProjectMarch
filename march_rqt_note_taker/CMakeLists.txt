cmake_minimum_required(VERSION 2.8.3)
project(march_rqt_note_taker)

find_package(catkin REQUIRED COMPONENTS
    roslint
)

catkin_python_setup()
catkin_package()

install(FILES plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/${PROJECT_NAME}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

file(GLOB_RECURSE lintfiles "src/*.py")
roslint_python(${lintfiles})
