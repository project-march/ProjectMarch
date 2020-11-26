// Copyright 2020 Project MARCH

#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "march_fake_sensor_data/FakeTemperatureData.hpp"

class FakeTemperatureDataNodeTest : public testing::Test {
    protected:
        void SetUp() override {
            rclcpp::init(0, nullptr);
            std::vector<float> weights = {0.1, 0.1, 0.1, 0.15, 0.15, 0.2, 0.2};
            node = new FakeTemperatureDataNode {"march_fake_temperature_node", std::move(weights)}; 
        }

        void TearDown() override {
            delete node;
            rclcpp::shutdown();
        }

        FakeTemperatureDataNode* node;
};

TEST_F(FakeTemperatureDataNodeTest, default_range)
{
    for (int i {0}; i < 10; ++i) {
        (*node).generate_new_temperature();
        ASSERT_EQ((*node).calculate_autoregression_temperature(), 0.0);
    }
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
