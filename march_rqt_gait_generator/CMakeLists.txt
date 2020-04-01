cmake_minimum_required(VERSION 2.8.3)
project(march_rqt_gait_generator)

find_package(catkin REQUIRED COMPONENTS
    march_shared_classes
    march_shared_resources
)

catkin_python_setup()
catkin_package()

install(FILES plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config launch resource
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/${PROJECT_NAME}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
    catkin_add_nosetests(test/run_tests.py)
endif()
