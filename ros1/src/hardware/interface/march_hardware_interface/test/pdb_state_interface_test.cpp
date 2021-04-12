// Copyright 2018 Project March.
#include "march_hardware_interface/march_pdb_state_interface.h"

#include <memory>

#include <gtest/gtest.h>

class PdbStateInterfaceTest : public ::testing::Test {
protected:
    std::unique_ptr<march::PowerDistributionBoard>
        power_distribution_board_read_;
    bool master_shutdown_allowed_command = false;
    bool all_high_voltage_on_off_command = true;
    PowerNetOnOffCommand power_net_on_off_command_;

    void SetUp() override
    {
        power_distribution_board_read_
            = std::make_unique<march::PowerDistributionBoard>(
                march::Slave(1, march::PdoInterfaceImpl::create(),
                    march::SdoInterfaceImpl::create()),
                NetMonitorOffsets(), NetDriverOffsets(), BootShutdownOffsets());
        master_shutdown_allowed_command = false;
        all_high_voltage_on_off_command = true;
        power_net_on_off_command_ = PowerNetOnOffCommand();
    }
};

TEST_F(PdbStateInterfaceTest, GeName)
{
    MarchPdbStateHandle marchPdbStateHandle("PDBhandle",
        power_distribution_board_read_.get(), &master_shutdown_allowed_command,
        &all_high_voltage_on_off_command, &power_net_on_off_command_);
    EXPECT_EQ("PDBhandle", marchPdbStateHandle.getName());
}

TEST_F(PdbStateInterfaceTest, GetPowerDistributionBoardEquals)
{
    MarchPdbStateHandle marchPdbStateHandle("PDBhandle",
        power_distribution_board_read_.get(), &master_shutdown_allowed_command,
        &all_high_voltage_on_off_command, &power_net_on_off_command_);
    EXPECT_TRUE(*power_distribution_board_read_
        == *marchPdbStateHandle.getPowerDistributionBoard());
}

TEST_F(PdbStateInterfaceTest, GetPowerDistributionBoardNotEqual)
{
    NetMonitorOffsets currentOffsets
        = NetMonitorOffsets(5, 9, 13, 17, 3, 2, 1, 4);
    NetDriverOffsets netDriverOffsets = NetDriverOffsets(4, 3, 2);
    BootShutdownOffsets stateOffsets = BootShutdownOffsets(0, 0, 1);
    march::PowerDistributionBoard pdb = march::PowerDistributionBoard(
        march::Slave(1, march::PdoInterfaceImpl::create(),
            march::SdoInterfaceImpl::create()),
        currentOffsets, netDriverOffsets, stateOffsets);
    MarchPdbStateHandle marchPdbStateHandle("PDBhandle",
        power_distribution_board_read_.get(), &master_shutdown_allowed_command,
        &all_high_voltage_on_off_command, &power_net_on_off_command_);
    EXPECT_FALSE(pdb == *marchPdbStateHandle.getPowerDistributionBoard());
}

TEST_F(PdbStateInterfaceTest, SetMasterShutdownAllowedTrue)
{
    MarchPdbStateHandle marchPdbStateHandle("PDBhandle",
        power_distribution_board_read_.get(), &master_shutdown_allowed_command,
        &all_high_voltage_on_off_command, &power_net_on_off_command_);

    EXPECT_FALSE(master_shutdown_allowed_command);
    marchPdbStateHandle.setMasterShutdownAllowed(true);
    EXPECT_TRUE(master_shutdown_allowed_command);
}
TEST_F(PdbStateInterfaceTest, SetMasterShutdownAllowedFalse)
{
    MarchPdbStateHandle marchPdbStateHandle("PDBhandle",
        power_distribution_board_read_.get(), &master_shutdown_allowed_command,
        &all_high_voltage_on_off_command, &power_net_on_off_command_);

    EXPECT_FALSE(master_shutdown_allowed_command);
    marchPdbStateHandle.setMasterShutdownAllowed(false);
    EXPECT_FALSE(master_shutdown_allowed_command);
}

TEST_F(PdbStateInterfaceTest, HighVoltageNetEnableDisable)
{
    MarchPdbStateHandle marchPdbStateHandle("PDBhandle",
        power_distribution_board_read_.get(), &master_shutdown_allowed_command,
        &all_high_voltage_on_off_command, &power_net_on_off_command_);

    EXPECT_TRUE(all_high_voltage_on_off_command);
    marchPdbStateHandle.setMasterShutdownAllowed(true);
    EXPECT_TRUE(all_high_voltage_on_off_command);
}
TEST_F(PdbStateInterfaceTest, TurnLowNetOn)
{
    MarchPdbStateHandle marchPdbStateHandle("PDBhandle",
        power_distribution_board_read_.get(), &master_shutdown_allowed_command,
        &all_high_voltage_on_off_command, &power_net_on_off_command_);

    // Check if the power net type is undefined
    EXPECT_EQ(PowerNetType(), power_net_on_off_command_.getType());
    marchPdbStateHandle.turnNetOnOrOff(PowerNetType("low_voltage"), true, 1);
    EXPECT_EQ(PowerNetType("low_voltage"), power_net_on_off_command_.getType());
    EXPECT_EQ(1, power_net_on_off_command_.getNetNumber());
    EXPECT_TRUE(power_net_on_off_command_.isOnOrOff());
}

TEST_F(PdbStateInterfaceTest, TurnLowNetOff)
{
    MarchPdbStateHandle marchPdbStateHandle("PDBhandle",
        power_distribution_board_read_.get(), &master_shutdown_allowed_command,
        &all_high_voltage_on_off_command, &power_net_on_off_command_);

    // Check if the power net type is undefined
    EXPECT_EQ(PowerNetType(), power_net_on_off_command_.getType());
    marchPdbStateHandle.turnNetOnOrOff(PowerNetType("low_voltage"), false, 1);
    EXPECT_EQ(PowerNetType("low_voltage"), power_net_on_off_command_.getType());
    EXPECT_EQ(1, power_net_on_off_command_.getNetNumber());
    EXPECT_FALSE(power_net_on_off_command_.isOnOrOff());
}
TEST_F(PdbStateInterfaceTest, TurnHighNetOn)
{
    MarchPdbStateHandle marchPdbStateHandle("PDBhandle",
        power_distribution_board_read_.get(), &master_shutdown_allowed_command,
        &all_high_voltage_on_off_command, &power_net_on_off_command_);

    // Check if the power net type is undefined
    EXPECT_EQ(PowerNetType(), power_net_on_off_command_.getType());
    marchPdbStateHandle.turnNetOnOrOff(PowerNetType("high_voltage"), true, 1);
    EXPECT_EQ(
        PowerNetType("high_voltage"), power_net_on_off_command_.getType());
    EXPECT_EQ(1, power_net_on_off_command_.getNetNumber());
    EXPECT_TRUE(power_net_on_off_command_.isOnOrOff());
}

TEST_F(PdbStateInterfaceTest, TurnHighNetOff)
{
    MarchPdbStateHandle marchPdbStateHandle("PDBhandle",
        power_distribution_board_read_.get(), &master_shutdown_allowed_command,
        &all_high_voltage_on_off_command, &power_net_on_off_command_);

    // Check if the power net type is undefined
    EXPECT_EQ(PowerNetType(), power_net_on_off_command_.getType());
    marchPdbStateHandle.turnNetOnOrOff(PowerNetType("high_voltage"), false, 1);
    EXPECT_EQ(
        PowerNetType("high_voltage"), power_net_on_off_command_.getType());
    EXPECT_EQ(1, power_net_on_off_command_.getNetNumber());
    EXPECT_FALSE(power_net_on_off_command_.isOnOrOff());
}
