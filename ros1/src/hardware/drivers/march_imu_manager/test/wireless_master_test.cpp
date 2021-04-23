// Copyright 2019 Project March
#include <gtest/gtest.h>

#include "march_imu_manager/wireless_master.h"

TEST(WirelessMasterTest, emptyRates)
{
    XsIntArray rates(/*sz=*/0, /*src=*/nullptr);
    const int rate = WirelessMaster::findClosestUpdateRate(
        rates, /*desired_update_rate=*/0);
    ASSERT_EQ(rate, 0);
}

TEST(WirelessMasterTest, oneRate)
{
    const int supportedRate = 60;
    XsIntArray rates(/*sz=*/1, &supportedRate);
    const int rate = WirelessMaster::findClosestUpdateRate(
        rates, /*desired_update_rate=*/0);
    ASSERT_EQ(rate, supportedRate);
}

TEST(WirelessMasterTest, matchingRate)
{
    const int supportedRates[3] = { 60, 80, 100 };
    XsIntArray rates(/*sz=*/3, supportedRates);
    const int rate
        = WirelessMaster::findClosestUpdateRate(rates, supportedRates[1]);
    ASSERT_EQ(rate, supportedRates[1]);
}

TEST(WirelessMasterTest, twoClosestRates)
{
    const int supportedRates[2] = { 10, 20 };
    XsIntArray rates(/*sz=*/2, supportedRates);
    const int rate = WirelessMaster::findClosestUpdateRate(
        rates, /*desired_update_rate=*/15);
    ASSERT_EQ(rate, supportedRates[0]);
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
