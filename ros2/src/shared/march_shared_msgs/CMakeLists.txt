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
  "msg/AfterLimitJointCommand.msg"
  "msg/Alive.msg"
  "msg/BatteryState.msg"
  "msg/CurrentGait.msg"
  "msg/CurrentIPDState.msg"
  "msg/CurrentState.msg"
  "msg/Error.msg"
  "msg/FeetHeightStamped.msg"
  "msg/FollowJointTrajectoryGoal.msg"
  "msg/FollowJointTrajectoryActionGoal.msg"
  "msg/FollowJointTrajectoryResult.msg"
  "msg/FollowJointTrajectoryActionResult.msg"
  "msg/FollowJointTrajectoryFeedback.msg"
  "msg/FollowJointTrajectoryActionFeedback.msg"
  "msg/FootPosition.msg"
  "msg/GaitInstruction.msg"
  "msg/GaitInstructionResponse.msg"
  "msg/GaitParameters.msg"
  "msg/HighVoltageState.msg"
  "msg/IpdInput.msg"
  "msg/JointMotorControllerState.msg"
  "msg/MotorControllerState.msg"
  "msg/MotorControllerStates.msg"
  "msg/LowVoltageState.msg"
  "msg/PowerDistributionBoardData.msg"
  "msg/PressureSole.msg"
  "msg/PressureSoleData.msg"
  "msg/PressureSolesData.msg"
  "msg/MpcArray.msg"
  "msg/MpcDiagnostics.msg"
  "msg/MpcStateVectors.msg"
  "msg/MpcTuning.msg"
  "msg/MpcJoint.msg"
  "msg/MpcMsg.msg"
  "msg/CenterOfMass.msg"
  "msg/RobotState.msg"
  "msg/IkSolverCommand.msg"
  "msg/GaitRequest.msg"
  "msg/GaitResponse.msg"
  "msg/GaitType.msg"
  "msg/PointStampedList.msg"
  "msg/TorqueStamped.msg"
  "msg/WeightStamped.msg"
  "msg/Feet.msg"
  "srv/CapturePointPose.srv"
  "srv/ContainsGait.srv"
  "srv/PublishTestDataset.srv"
  "srv/PossibleGaits.srv"
  "srv/SetGaitVersion.srv"
  "srv/SetObstacleSizeRampLike.srv"
  "srv/SetObstacleSizeBlockLike.srv"
  "srv/GetGaitParameters.srv"
  "srv/GetMoveItTrajectory.srv"
  "srv/GetJointNames.srv"
  "srv/SetObstacleSize.srv"
  "srv/GaitCommand.srv"
  "srv/RequestFootsteps.srv"
  "srv/RequestGait.srv"
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs rosgraph_msgs
        trajectory_msgs control_msgs actionlib_msgs sensor_msgs
 )

install(
  FILES mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME})

ament_package()
