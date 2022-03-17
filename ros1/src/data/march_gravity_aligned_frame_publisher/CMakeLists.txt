cmake_minimum_required(VERSION 3.0.2)
project(march_gravity_aligned_frame_publisher)

find_package(catkin)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS)

catkin_install_python(PROGRAMS scripts/march_gravity_aligned_frame_publisher
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
