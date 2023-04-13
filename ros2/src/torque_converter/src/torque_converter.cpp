#include "torque_converter/torque_converter.hpp"
#include <cstdio>

#include <chrono>
#include <cstdio>
#include <string>
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

TorqueConverter::TorqueConverter()
    : Node("torque_converter")
{
    m_trajectory_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
        "/torque_trajectory", 10, std::bind(&TorqueConverter::trajectory_subscriber_callback, this, _1));
}

void TorqueConverter::trajectory_subscriber_callback(sensor_msgs::msg::JointState::SharedPtr msg)
{
}
