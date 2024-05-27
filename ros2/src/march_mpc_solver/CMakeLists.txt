cmake_minimum_required(VERSION 3.5)
project(march_mpc_solver)

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
find_package(ament_cmake_gmock REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(march_shared_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)


set(acados_include "${PROJECT_SOURCE_DIR}/../libraries/acados/include")
set(acados_lib "${PROJECT_SOURCE_DIR}/../libraries/acados/lib")

# Necessary for testing
add_library(${PROJECT_NAME}_testlib
  STATIC
  src/zmp_mpc_solver_node.cpp
  src/zmp_mpc_solver.cpp
  include/${PROJECT_NAME}/zmp_mpc_solver.hpp
  include/${PROJECT_NAME}/zmp_mpc_solver_node.hpp
)

ament_target_dependencies(
  ${PROJECT_NAME}_testlib
  "pluginlib"
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "march_shared_msgs"
)

# 
add_executable(zmp_mpc_solver
src/zmp_mpc_solver_node.cpp
src/zmp_mpc_solver.cpp
include/${PROJECT_NAME}/zmp_mpc_solver.hpp
include/${PROJECT_NAME}/zmp_mpc_solver_node.hpp
)
ament_target_dependencies(zmp_mpc_solver rclcpp march_shared_msgs trajectory_msgs std_msgs)
target_include_directories(zmp_mpc_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  zmp_mpc_solver
  "pluginlib"
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "march_shared_msgs"
  "visualization_msgs"
)




## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        ${acados_include}
        ${acados_include}/blasfeo/include/
        ${acados_include}/hpipm/include/
        ${acados_include}/acados/
        ${acados_include}/qpOASES_e/
        ${acados_include}/acados_c/
        ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/c_generated_code/

)


target_link_libraries(zmp_mpc_solver ${acados_lib}/libblasfeo.so)
target_link_libraries(zmp_mpc_solver ${acados_lib}/libhpipm.so)
target_link_libraries(zmp_mpc_solver ${acados_lib}/libqpOASES_e.so)
target_link_libraries(zmp_mpc_solver ${acados_lib}/libqpOASES_e.so.3.1)
target_link_libraries(zmp_mpc_solver ${acados_lib}/libacados.so)
target_link_libraries(zmp_mpc_solver 
${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/c_generated_code/libacados_ocp_solver_ZMP_pendulum_ode.so
)

# FOR TESTING
target_link_libraries(${PROJECT_NAME}_testlib ${acados_lib}/libblasfeo.so)
target_link_libraries(${PROJECT_NAME}_testlib ${acados_lib}/libhpipm.so)
target_link_libraries(${PROJECT_NAME}_testlib ${acados_lib}/libqpOASES_e.so)
target_link_libraries(${PROJECT_NAME}_testlib ${acados_lib}/libqpOASES_e.so.3.1)
target_link_libraries(${PROJECT_NAME}_testlib ${acados_lib}/libacados.so)
target_link_libraries(${PROJECT_NAME}_testlib 
${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/c_generated_code/libacados_ocp_solver_ZMP_pendulum_ode.so
)
# 

install(TARGETS zmp_mpc_solver
  DESTINATION lib/${PROJECT_NAME})

install(FILES ${acados_lib}/libblasfeo.so
  DESTINATION lib)

install(FILES ${acados_lib}/libacados.so
  DESTINATION lib)

install(FILES ${acados_lib}/libqpOASES_e.so
  DESTINATION lib)

  install(FILES ${acados_lib}/libqpOASES_e.so.3.1
  DESTINATION lib)

install(FILES ${acados_lib}/libhpipm.so
  DESTINATION lib)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/c_generated_code/libacados_ocp_solver_ZMP_pendulum_ode.so
  DESTINATION lib)



if(BUILD_TESTING)
  set(ament_cmake_copyright_FOUND TRUE)  
  #find_package(ament_lint_auto REQUIRED)
  ament_add_gmock(${PROJECT_NAME}_test
    test/test_runner.cpp
    test/zmp_mpc_test.cpp
    # include
    )
  
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME}_testlib)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(
    ${PROJECT_NAME}_test
    "pluginlib"
    "rclcpp"
    "geometry_msgs"
    "nav_msgs"
    "visualization_msgs")
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
endif()

ament_package()
