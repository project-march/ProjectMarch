// Copyright 2020 Project MARCH

#include "march_fake_sensor_data/UniformDistribution.hpp"
#include <gtest/gtest.h>

TEST(UniformDistributionTest, generate_within_bounds)
{
    int lower_bound = -5;
    int upper_bound = 5;
    UniformDistribution distribution { lower_bound, upper_bound };
    for (int i { 0 }; i < 30; ++i) {
        auto number = distribution.get_random_number();
        ASSERT_LE(lower_bound, number);
        ASSERT_GE(upper_bound, number);
    }
}

TEST(UniformDistributionTest, valid_change_range)
{
    UniformDistribution distribution { -100, 100 };
    distribution.set_range(-1, 1);
    for (int i { 0 }; i < 10; ++i) {
        auto number = distribution.get_random_number();
        ASSERT_LE(-1, number);
        ASSERT_GE(1, number);
    }
}

TEST(UniformDistributionTest, inverted_range)
{
    int lower_bound = -3;
    int upper_bound = 3;
    UniformDistribution distribution { upper_bound, lower_bound };
    for (int i { 0 }; i < 30; ++i) {
        auto number = distribution.get_random_number();
        ASSERT_LE(lower_bound, number);
        ASSERT_GE(upper_bound, number);
    }
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
