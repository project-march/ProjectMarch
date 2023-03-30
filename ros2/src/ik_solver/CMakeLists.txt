cmake_minimum_required(VERSION 3.5)
project(ik_solver)

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

# We need the library locations from ACADOS, as they contain hpipm and blasfeo
set(acados_include "${PROJECT_SOURCE_DIR}/../libraries/acados/include")
set(acados_lib "${PROJECT_SOURCE_DIR}/../libraries/acados/lib")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(march_description REQUIRED)
find_package(march_shared_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_packagE(pluginlib REQUIRED)
find_package(pinocchio REQUIRED)
find_package(urdf REQUIRED)
find_package(labrob_qpsolvers REQUIRED)
include_directories(include)

add_library(${PROJECT_NAME}
  STATIC
  include/${PROJECT_NAME}/ik_solver.hpp
  src/ik_solver.cpp
  include/${PROJECT_NAME}/ik_solver_node.hpp
  src/ik_solver_node.cpp
  include/${PROJECT_NAME}/utils.hpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  "rclcpp"
  "geometry_msgs"
  "march_description"
  "march_shared_msgs"
  labrob_qpsolvers
  pinocchio
  urdf
  Eigen3
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(ik_solver_node 
src/ik_solver_node.cpp 
src/ik_solver.cpp 
include/${PROJECT_NAME}/utils.hpp
include/${PROJECT_NAME}/ik_solver.hpp
include/${PROJECT_NAME}/ik_solver_node.hpp)
target_include_directories(ik_solver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# target_link_libraries (ik_solver_node Eigen3::Eigen pinocchio::pinocchio)
ament_target_dependencies(
  ik_solver_node
  pinocchio
  Eigen3
  labrob_qpsolvers
  urdf
  "rclcpp"
  "geometry_msgs"
  "march_description"
  "march_shared_msgs"
)

# 
include_directories(
        ${acados_include}
        ${acados_include}/blasfeo/include/
        ${acados_include}/hpipm/include/
        ${acados_include}/acados_c/
)
# Link libraries
target_link_libraries(ik_solver_node ${acados_lib}/libblasfeo.so)
target_link_libraries(ik_solver_node ${acados_lib}/libhpipm.so)

target_link_libraries(${PROJECT_NAME} ${acados_lib}/libblasfeo.so)
target_link_libraries(${PROJECT_NAME} ${acados_lib}/libhpipm.so)
# 

install(TARGETS ik_solver_node
EXPORT export_${PROJECT_NAME}
DESTINATION lib/${PROJECT_NAME})

install(FILES ${acados_lib}/libblasfeo.so
  DESTINATION lib)

  install(FILES ${acados_lib}/libhpipm.so
  DESTINATION lib)

# Install launch files.
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)




#Export
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
file(COPY test/urdf/march_test_urdf.urdf DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/test/urdf)
  ament_add_gmock(${PROJECT_NAME}_test
  test/ik_solver_test.cpp
  test/test_runner.cpp
  # include
  )
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
  target_include_directories(${PROJECT_NAME}_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include>)

  # target_include_directories(${TEST_MY_COOL_PKG} PRIVATE "test/include" "include")
  ament_target_dependencies(
    ${PROJECT_NAME}_test 
    pinocchio
    Eigen3
    labrob_qpsolvers
    urdf
    "rclcpp"
    "geometry_msgs" 
    "march_shared_msgs")
endif()

ament_package()
