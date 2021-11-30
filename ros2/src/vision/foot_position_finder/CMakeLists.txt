cmake_minimum_required(VERSION 3.5)
project(foot_position_finder)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find_package(ament_cmake REQUIRED)
find_package(PCL 1.7.2 REQUIRED)
find_package(realsense2 REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/preprocessor.cpp
    src/foot_position_finder.cpp
)

add_executable(foot_position_finder_node src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})
target_link_libraries(foot_position_finder_node ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node 
  DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()
