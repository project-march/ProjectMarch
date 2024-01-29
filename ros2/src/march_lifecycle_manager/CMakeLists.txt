cmake_minimum_required(VERSION 3.5)
project(march_lifecycle_manager)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# add_library(march_lifecycle_manager STATIC
#   src/lifecycle_manager.cpp
# )

add_executable(lifecycle_manager
  src/march_lifecycle_manager.cpp
)

target_include_directories(lifecycle_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(lifecycle_manager
  ${library_name}
)

ament_target_dependencies(lifecycle_manager
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  std_msgs
  std_srvs
)

install(TARGETS lifecycle_manager
  DESTINATION lib/march_lifecycle_manager
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()

#   find_package(ament_cmake_gtest REQUIRED)
#   find_package(ament_cmake_pytest REQUIRED)

#   add_subdirectory(test)
# endif()

# ament_export_include_directories(include)
# ament_export_libraries(${library_name})
# ament_export_dependencies(${dependencies})

ament_package()
