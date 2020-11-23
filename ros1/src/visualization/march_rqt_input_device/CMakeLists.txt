cmake_minimum_required(VERSION 2.8.3)
project(march_rqt_input_device)

find_package(catkin REQUIRED COMPONENTS march_shared_resources std_msgs)

catkin_python_setup()
catkin_package(CATKIN_DEPENDS march_shared_resources std_msgs)

install(FILES plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch resource
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/${PROJECT_NAME}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
