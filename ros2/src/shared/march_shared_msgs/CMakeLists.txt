cmake_minimum_required(VERSION 3.5)
project(march_shared_msgs)

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
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Run the Python script to generate ExoMode.msg
find_package(Python3 COMPONENTS Interpreter REQUIRED)

set(PYTHON_SCRIPT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/generate/generate_exo_mode_message.py")

add_custom_target(
    run_python_script ALL
    COMMAND ${Python3_EXECUTABLE} ${PYTHON_SCRIPT_PATH}
    COMMENT "Generating ExoMode.msg from JSON file"
    VERBATIM
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Alive.msg"
  "msg/BatteryState.msg"
  "msg/CenterOfMass.msg"
  "msg/CurrentGait.msg"
  "msg/CurrentState.msg"
  "msg/DynamicalJointState.msg"
  "msg/Error.msg"
  "msg/Feet.msg"
  "msg/ExoMode.msg"
  "msg/ExoModeAndJoint.msg"
  "msg/ExoModeArray.msg"
  "msg/FeetHeightStamped.msg"
  "msg/FootPosition.msg"
  "msg/FootStepOutput.msg"
  "msg/GaitInstruction.msg"
  "msg/GaitRequest.msg"
  "msg/GaitResponse.msg"
  "msg/GaitType.msg"
  "msg/HighVoltageState.msg"
  "msg/IksCommand.msg"
  "msg/IksFootPositions.msg"
  "msg/IksTaskStatus.msg"
  "msg/IksStatus.msg"
  "msg/JointEfforts.msg"
  "msg/JointMotorControllerState.msg"
  "msg/LowVoltageState.msg"
  "msg/MotorControllerStates.msg"
  "msg/NodeJacobian.msg"
  "msg/PidValues.msg"
  "msg/PowerDistributionBoardData.msg"
  "msg/RobotState.msg"
  "msg/TorqueStamped.msg"
  "msg/StateEstimation.msg"
  "msg/StateEstimatorVisualization.msg"
  "msg/WeightStamped.msg"
  "srv/GetCurrentStanceLeg.srv"
  "srv/GetExoModeArray.srv"
  "srv/GetJointNames.srv"
  "srv/GetNodePosition.srv"
  "srv/GetNodeJacobian.srv"
  "srv/GaitCommand.srv"
  "srv/GetCurrentJointPositions.srv"
  "srv/GetTaskReport.srv"
  "srv/RequestFootsteps.srv"
  "srv/RequestGait.srv"
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs rosgraph_msgs
        trajectory_msgs control_msgs actionlib_msgs sensor_msgs
 )

install(
  FILES mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME})

ament_package()
