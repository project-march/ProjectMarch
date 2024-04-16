#ifndef __clang_analyzer__
// NOLINTBEGIN
#pragma once
#include "mock_absolute_encoder.h"
#include "mock_incremental_encoder.h"
#include "mock_slave.h"
#include "rclcpp/rclcpp.hpp"

#include "march_hardware/ethercat/sdo_interface.h"
#include "march_hardware/motor_controller/odrive/odrive.h"
#include "march_hardware/motor_controller/odrive/odrive_state.h"

#include <memory>

#include <gmock/gmock.h>
#include <march_logger_cpp/ros_logger.hpp>

class MockODrive : public march::ODrive {
public:
    std::shared_ptr<march_logger::BaseLogger> logger_
        = std::make_shared<march_logger::RosLogger>("march_hardware_builder");

    MockODrive()
        : ODrive(MockSlave(), march::ODriveAxis::One, std::make_unique<MockAbsoluteEncoder>(),
            std::make_unique<MockIncrementalEncoder>(), march::ActuationMode::unknown, true, 100, true, logger_)
    {
    }

    MOCK_METHOD0(getState, std::unique_ptr<march::MotorControllerState>());

    MOCK_METHOD0(prepareActuation, std::chrono::nanoseconds());

    MOCK_METHOD0(isIncrementalEncoderMorePrecise, bool());

    MOCK_METHOD0(getIncrementalPositionUnchecked, float());
    MOCK_METHOD0(getAbsolutePositionUnchecked, float());
    MOCK_METHOD0(getIncrementalVelocityUnchecked, float());
    MOCK_METHOD0(getAbsoluteVelocityUnchecked, float());

    MOCK_METHOD0(getMotorControllerVoltage, float());
    MOCK_METHOD0(getMotorVoltage, float());
    MOCK_METHOD0(getMotorCurrent, float());

    MOCK_METHOD1(actuateRadians, void(float));
    MOCK_METHOD1(actuateTorque, void(float));

    MOCK_METHOD2(initSdo, bool(march::SdoSlaveInterface& sdo, int cycle_time));
    MOCK_METHOD1(resetSlave, void(march::SdoSlaveInterface&));
};
// NOLINTEND
#endif
