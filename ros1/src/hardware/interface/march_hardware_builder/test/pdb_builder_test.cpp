// Copyright 2019 Project March.
#include <string>

#include <gtest/gtest.h>
#include <march_hardware_builder/hardware_builder.h>
#include <march_hardware_builder/hardware_config_exceptions.h>
#include <ros/package.h>

class PowerDistributionBoardBuilderTest : public ::testing::Test {
protected:
    std::string base_path;
    march::PdoInterfacePtr pdo_interface;
    march::SdoInterfacePtr sdo_interface;

    void SetUp() override
    {
        base_path = ros::package::getPath("march_hardware_builder")
                        .append(/*__s=*/"/test/yaml/powerdistributionboard");
        this->pdo_interface = march::PdoInterfaceImpl::create();
        this->sdo_interface = march::SdoInterfaceImpl::create();
    }

    std::string fullPath(const std::string& relativePath)
    {
        return this->base_path.append(relativePath);
    }
};

TEST_F(PowerDistributionBoardBuilderTest, ValidPowerDistributionBoard)
{
    std::string fullPath = this->fullPath("/power_distribution_board.yaml");
    YAML::Node config = YAML::LoadFile(fullPath);

    auto createdPowerDistributionBoard
        = HardwareBuilder::createPowerDistributionBoard(
            config, this->pdo_interface, this->sdo_interface);
    NetMonitorOffsets netMonitoringOffsets(
        /*powerDistributionBoardCurrentByteOffset=*/5,
        /*lowVoltageNet1CurrentByteOffset=*/9,
        /*lowVoltageNet2CurrentByteOffset=*/13,
        /*highVoltageNetCurrentByteOffset=*/17, /*lowVoltageStateByteOffset=*/3,
        /*highVoltageOvercurrentTriggerByteOffset=*/2, /*highVoltageEnabled=*/1,
        /*highVoltageStateByteOffset=*/4);
    NetDriverOffsets netDriverOffsets(/*lowVoltageNetOnOff=*/4,
        /*highVoltageNetOnOff=*/3, /*highVoltageNetEnableDisable=*/2);
    BootShutdownOffsets bootShutdownOffsets(/*masterOkByteOffset=*/0,
        /*shutdownByteOffset=*/0, /*shutdownAllowedByteOffset=*/1);
    march::PowerDistributionBoard powerDistributionBoard
        = march::PowerDistributionBoard(
            march::Slave(
                /*slave_index=*/1, this->pdo_interface, this->sdo_interface),
            netMonitoringOffsets, netDriverOffsets, bootShutdownOffsets);

    ASSERT_EQ(powerDistributionBoard, *createdPowerDistributionBoard);
}

TEST_F(PowerDistributionBoardBuilderTest, NoConfig)
{
    YAML::Node config;
    ASSERT_EQ(nullptr,
        HardwareBuilder::createPowerDistributionBoard(
            config["pdb"], this->pdo_interface, this->sdo_interface));
}

TEST_F(PowerDistributionBoardBuilderTest, MissingSlaveIndex)
{
    std::string fullPath
        = this->fullPath("/power_distribution_board_missing_slave_index.yaml");
    YAML::Node config = YAML::LoadFile(fullPath);
    ASSERT_THROW(HardwareBuilder::createPowerDistributionBoard(
                     config, this->pdo_interface, this->sdo_interface),
        MissingKeyException);
}

TEST_F(PowerDistributionBoardBuilderTest, MissingHighVoltageStateIndex)
{
    std::string fullPath = this->fullPath(
        "/power_distribution_board_missing_high_voltage_state_index.yaml");
    YAML::Node config = YAML::LoadFile(fullPath);
    ASSERT_THROW(HardwareBuilder::createPowerDistributionBoard(
                     config, this->pdo_interface, this->sdo_interface),
        MissingKeyException);
}

TEST_F(PowerDistributionBoardBuilderTest, NegativeOffset)
{
    std::string fullPath
        = this->fullPath("/power_distribution_board_negative_offset.yaml");
    YAML::Node config = YAML::LoadFile(fullPath);
    ASSERT_THROW(HardwareBuilder::createPowerDistributionBoard(
                     config, this->pdo_interface, this->sdo_interface),
        std::runtime_error);
}
