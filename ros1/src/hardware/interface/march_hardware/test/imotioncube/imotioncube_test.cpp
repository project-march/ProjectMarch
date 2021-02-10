// Copyright 2018 Project March.
#include "../mocks/mock_absolute_encoder.h"
#include "../mocks/mock_incremental_encoder.h"
#include "../mocks/mock_slave.h"

#include <march_hardware/motor_controller/imotioncube/imotioncube.h>
#include <march_hardware/error/hardware_exception.h>

#include <memory>
#include <stdexcept>
#include <utility>

#include <gtest/gtest.h>

class IMotionCubeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    this->mock_absolute_encoder = std::make_unique<MockAbsoluteEncoder>();
    this->mock_incremental_encoder = std::make_unique<MockIncrementalEncoder>();
  }

  MockPdoInterfacePtr mock_pdo = std::make_shared<MockPdoInterface>();
  MockSdoInterfacePtr mock_sdo = std::make_shared<MockSdoInterface>();
  MockSlave mock_slave = MockSlave(this->mock_pdo, this->mock_sdo);
  std::unique_ptr<MockAbsoluteEncoder> mock_absolute_encoder;
  std::unique_ptr<MockIncrementalEncoder> mock_incremental_encoder;
};

TEST_F(IMotionCubeTest, NoAbsoluteEncoder)
{
  ASSERT_THROW(
      march::IMotionCube(mock_slave, nullptr, std::move(this->mock_incremental_encoder), march::ActuationMode::unknown),
      std::invalid_argument);
}

TEST_F(IMotionCubeTest, NoIncrementalEncoder)
{
  ASSERT_THROW(
      march::IMotionCube(mock_slave, std::move(this->mock_absolute_encoder), nullptr, march::ActuationMode::unknown),
      std::invalid_argument);
}

TEST_F(IMotionCubeTest, SlaveIndexOne)
{
  march::IMotionCube imc(mock_slave, std::move(this->mock_absolute_encoder), std::move(this->mock_incremental_encoder),
                         march::ActuationMode::unknown);
  ASSERT_EQ(1, imc.getSlaveIndex());
}

TEST_F(IMotionCubeTest, NoActuationMode)
{
  march::IMotionCube imc(mock_slave, std::move(this->mock_absolute_encoder), std::move(this->mock_incremental_encoder),
                         march::ActuationMode::unknown);
  ASSERT_THROW(imc.actuateRad(1), march::error::HardwareException);
}

TEST_F(IMotionCubeTest, ActuationModeTorqueActuateRad)
{
  march::IMotionCube imc(mock_slave, std::move(this->mock_absolute_encoder), std::move(this->mock_incremental_encoder),
                         march::ActuationMode::torque);

  ASSERT_EQ(march::ActuationMode::torque, imc.getActuationMode().getValue());
  ASSERT_THROW(imc.actuateRad(1), march::error::HardwareException);
}

TEST_F(IMotionCubeTest, ActuationModePositionActuateTorque)
{
  march::IMotionCube imc(mock_slave, std::move(this->mock_absolute_encoder), std::move(this->mock_incremental_encoder),
                         march::ActuationMode::position);

  ASSERT_THROW(imc.actuateTorque(1), march::error::HardwareException);
}

TEST_F(IMotionCubeTest, OperationEnabledWithoutActuationMode)
{
  march::IMotionCube imc(mock_slave, std::move(this->mock_absolute_encoder), std::move(this->mock_incremental_encoder),
                         march::ActuationMode::unknown);
  ASSERT_THROW(imc.goToOperationEnabled(), march::error::HardwareException);
}
