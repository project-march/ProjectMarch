// Copyright 2020 Project MARCH

#include "march_fake_sensor_data/FakeTemperatureData.hpp"
#include "rclcpp/rclcpp.hpp"
#include <gtest/gtest.h>

class FakeTemperatureDataNodeTest : public testing::Test {
protected:
    void SetUp() override
    {
        rclcpp::init(/*argc=*/0, /*argv=*/nullptr);
        std::vector<float> weights = { 0.1, 0.1, 0.1, 0.15, 0.15, 0.2, 0.2 };
        node = new FakeTemperatureDataNode { "march_fake_temperature_node",
            std::move(weights) };
    }

    void TearDown() override
    {
        delete node;
        rclcpp::shutdown();
    }

    FakeTemperatureDataNode* node {};
};

TEST_F(FakeTemperatureDataNodeTest, test_default_range)
{
    for (int i { 0 }; i < 10; ++i) {
        (*node).generate_new_temperature();
        ASSERT_EQ((*node).calculate_autoregression_temperature(), 0.0);
    }
}

TEST_F(FakeTemperatureDataNodeTest, test_change_range)
{
    (*node).set_range(/*minimum_temperature=*/-10, /*maximum_temperature=*/10);
    for (int i { 0 }; i < 10; ++i) {
        (*node).generate_new_temperature();
        ASSERT_LE((*node).calculate_autoregression_temperature(), 10);
        ASSERT_GE((*node).calculate_autoregression_temperature(), -10);
    }

    (*node).set_range(/*minimum_temperature=*/-2, /*maximum_temperature=*/2);
    // "flush" the previous values
    for (int i { 0 }; i < 10; ++i) {
        (*node).generate_new_temperature();
    }

    for (int i { 0 }; i < 10; ++i) {
        (*node).generate_new_temperature();
        ASSERT_LE((*node).calculate_autoregression_temperature(), 2);
        ASSERT_GE((*node).calculate_autoregression_temperature(), -2);
    }
}

TEST_F(FakeTemperatureDataNodeTest, test_autoregression)
{
    // Tear down the test fixture because this function will do this manually.
    TearDown();

    // This test is based on the weights defined in the setup. Because the
    // initial temperatures are known to be 0, this checks if the autoregression
    // approaches the boundaries of the random range as expected.
    // std::vector<float> weights = {0.1, 0.1, 0.1, 0.15, 0.15, 0.2, 0.2};
    int bound = 5;

    for (int i { 0 }; i < 100; ++i) {
        SetUp();
        (*node).set_range(-bound, bound);
        (*node).generate_new_temperature();
        EXPECT_LE((*node).calculate_autoregression_temperature(), 0.2 * bound);
        EXPECT_GE((*node).calculate_autoregression_temperature(), 0.2 * -bound);
        (*node).generate_new_temperature();
        EXPECT_LE((*node).calculate_autoregression_temperature(), 0.4 * bound);
        EXPECT_GE((*node).calculate_autoregression_temperature(), 0.4 * -bound);
        (*node).generate_new_temperature();
        EXPECT_LE((*node).calculate_autoregression_temperature(), 0.55 * bound);
        EXPECT_GE(
            (*node).calculate_autoregression_temperature(), 0.55 * -bound);
        (*node).generate_new_temperature();
        EXPECT_LE((*node).calculate_autoregression_temperature(), 0.70 * bound);
        EXPECT_GE(
            (*node).calculate_autoregression_temperature(), 0.70 * -bound);
        (*node).generate_new_temperature();
        EXPECT_LE((*node).calculate_autoregression_temperature(), 0.80 * bound);
        EXPECT_GE(
            (*node).calculate_autoregression_temperature(), 0.80 * -bound);
        (*node).generate_new_temperature();
        EXPECT_LE((*node).calculate_autoregression_temperature(), 0.90 * bound);
        EXPECT_GE(
            (*node).calculate_autoregression_temperature(), 0.90 * -bound);
        (*node).generate_new_temperature();
        EXPECT_LE((*node).calculate_autoregression_temperature(), bound);
        EXPECT_GE((*node).calculate_autoregression_temperature(), -bound);
        TearDown();
    }

    // Reset the test fixture to its "original" state.
    SetUp();
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
