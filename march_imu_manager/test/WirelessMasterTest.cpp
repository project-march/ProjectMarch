// Copyright 2019 Project March
#include <gtest/gtest.h>

#include "march_imu_manager/WirelessMaster.h"

TEST(WirelessMasterTest, emptyRates)
{
    XsIntArray rates(0, nullptr);
    const int rate = WirelessMaster::findClosestUpdateRate(rates, 0);
    ASSERT_EQ(rate, 0);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
