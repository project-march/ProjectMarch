// Copyright 2019 Project March.

#include "gtest/gtest.h"
#include "ros/ros.h"
#include <gmock/gmock.h>
#include <ros/package.h>
#include <march_hardware_builder/HardwareConfigExceptions.h>
#include <march_hardware_builder/HardwareBuilder.h>

using ::testing::AtLeast;
using ::testing::Return;

class PowerDistributionBoardTest : public ::testing::Test
{
protected:
  std::string base_path;
  HardwareBuilder hardwareBuilder;

  void SetUp() override
  {
    base_path = ros::package::getPath("march_hardware_builder").append("/test/yaml/powerdistributionboard");
  }

  std::string fullPath(const std::string& relativePath)
  {
    return this->base_path.append(relativePath);
  }
};

TEST_F(PowerDistributionBoardTest, ValidPowerDistributionBoard)
{
  std::string fullPath = this->fullPath("/power_distribution_board.yaml");
  YAML::Node config = YAML::LoadFile(fullPath);

  march4cpp::PowerDistributionBoard createdPowerDistributionBoard =
      hardwareBuilder.createPowerDistributionBoard(config);
  NetMonitorOffsets netMonitoringOffsets(5, 9, 13, 17, 3, 2, 1, 4);
  NetDriverOffsets netDriverOffsets(4, 3, 2);
  BootShutdownOffsets bootShutdownOffsets(0, 0, 1);
  march4cpp::PowerDistributionBoard powerDistributionBoard =
      march4cpp::PowerDistributionBoard(1, netMonitoringOffsets, netDriverOffsets, bootShutdownOffsets);

  ASSERT_EQ(powerDistributionBoard, createdPowerDistributionBoard);
}

TEST_F(PowerDistributionBoardTest, MissingSlaveIndex)
{
  std::string fullPath = this->fullPath("/power_distribution_board_missing_slave_index.yaml");
  YAML::Node config = YAML::LoadFile(fullPath);
  ASSERT_THROW(hardwareBuilder.createPowerDistributionBoard(config), MissingKeyException);
}

TEST_F(PowerDistributionBoardTest, MissingHighVoltageStateIndex)
{
  std::string fullPath = this->fullPath("/power_distribution_board_missing_high_voltage_state_index.yaml");
  YAML::Node config = YAML::LoadFile(fullPath);
  ASSERT_THROW(hardwareBuilder.createPowerDistributionBoard(config), MissingKeyException);
}

TEST_F(PowerDistributionBoardTest, NegativeOffset)
{
  std::string fullPath = this->fullPath("/power_distribution_board_negative_offset.yaml");
  YAML::Node config = YAML::LoadFile(fullPath);
  ASSERT_THROW(hardwareBuilder.createPowerDistributionBoard(config), std::runtime_error);
}
