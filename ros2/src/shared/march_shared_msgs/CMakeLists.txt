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


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/hardware/AfterLimitJointCommand.msg"
  "msg/input_device/Alive.msg"
  "msg/gait/CurrentGait.msg"
  "msg/gait/GaitParameters.msg"
  "msg/gait/CurrentState.msg"
  "msg/Error.msg"
  "msg/joint_trajectory/FollowJointTrajectoryGoal.msg"
  "msg/joint_trajectory/FollowJointTrajectoryActionGoal.msg"
  "msg/joint_trajectory/FollowJointTrajectoryResult.msg"
  "msg/joint_trajectory/FollowJointTrajectoryActionResult.msg"
  "msg/joint_trajectory/FollowJointTrajectoryFeedback.msg"
  "msg/joint_trajectory/FollowJointTrajectoryActionFeedback.msg"
  "msg/joint_trajectory/GoalID.msg"
  "msg/joint_trajectory/GoalStatus.msg"
  "msg/input_device/GaitInstruction.msg"
  "msg/input_device/GaitInstructionResponse.msg"
  "msg/hardware/HighVoltageNet.msg"
  "msg/hardware/ImcState.msg"
  "msg/hardware/LowVoltageNet.msg"
  "msg/hardware/PowerDistributionBoardState.msg"
  "msg/pressure_sole/PressureSole.msg"
  "msg/pressure_sole/PressureSoleData.msg"
  "msg/pressure_sole/PressureSolesData.msg"
  "srv/balance/CapturePointPose.srv"
  "srv/gait/ContainsGait.srv"
  "srv/gait/PossibleGaits.srv"
  "srv/gait/SetGaitVersion.srv"
  "srv/gait/GetJointNames.srv"
  "srv/simulation/SetObstacleSizeRampLike.srv"
  "srv/simulation/SetObstacleSizeBlockLike.srv"
  "srv/balance/GetMoveItTrajectory.srv"
  "srv/bridge/GetParamStringList.srv"
  "srv/bridge/GetParamString.srv"
  "srv/bridge/GetParamBool.srv"
  "srv/bridge/GetParamInt.srv"
  "srv/bridge/GetParamFloat.srv"
  "srv/bridge/SetParamStringList.srv"
  "srv/bridge/SetParamString.srv"
  "srv/bridge/SetParamBool.srv"
  "srv/bridge/SetParamInt.srv"
  "srv/bridge/SetParamFloat.srv"
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs rosgraph_msgs
        trajectory_msgs control_msgs actionlib_msgs sensor_msgs
 )

install(
  FILES mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME})

ament_package()
