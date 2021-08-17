cmake_minimum_required(VERSION 3.0.2)
project(march_shared_msgs)

find_package(catkin REQUIRED COMPONENTS
    control_msgs
    geometry_msgs
    message_generation
    std_msgs
    trajectory_msgs
    sensor_msgs
)

add_message_files(
    FILES
    AfterLimitJointCommand.msg
    Alive.msg
    BatteryState.msg
    CurrentGait.msg
    CurrentIPDState.msg
    CurrentState.msg
    Error.msg
    GaitInstruction.msg
    GaitInstructionResponse.msg
    GaitParameters.msg
    HighVoltageState.msg
    MotorControllerState.msg
    JointValues.msg
    LowVoltageState.msg
    PowerDistributionBoardData.msg
    PressureSole.msg
    PressureSoleData.msg
    PressureSolesData.msg
    MpcArray.msg
    MpcStateVectors.msg
    MpcTuning.msg
    MpcDiagnostics.msg
    MpcJoint.msg
    MpcMsg.msg
)

add_service_files(
    FILES
    CapturePointPose.srv
    ChangeComLevel.srv
    ContainsGait.srv
    GetGaitParameters.srv
    GetMoveItTrajectory.srv
    GetPossibleComLevels.srv
    PossibleGaits.srv
    PublishTestDataset.srv
    SetGaitVersion.srv
    SetObstacleSizeRampLike.srv
    SetObstacleSizeBlockLike.srv
)

generate_messages(
    DEPENDENCIES
    control_msgs
    std_msgs
    trajectory_msgs
    sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS
    control_msgs
    geometry_msgs
    message_runtime
    std_msgs
    trajectory_msgs
    sensor_msgs
)
