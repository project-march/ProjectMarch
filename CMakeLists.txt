cmake_minimum_required(VERSION 2.8.3)
project(march_tutorials)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        march_shared_resources
)


set(THIS_PACKAGE_INCLUDE_DIRS
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
  DEPENDS
)

###########
## Build ##
###########

include_directories(${THIS_PACKAGE_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# Add subdirectories with a CMakeLists.txt here
#add_subdirectory(doc/<example>)
