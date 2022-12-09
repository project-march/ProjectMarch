//
// Created by Marco on 6-12-22.
//

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "march_utility/logger_colors.hpp"
#include "march_hardware_interface/march_rviz_system_interface.hpp"
#include "march_hardware_interface/read_mujoco_info_node.hpp"

using std::placeholders::_1;

using namespace march_hardware_interface;

ReadMujocoInfo::ReadMujocoInfo()
    : Node("read_mujoco_info_node")
{
    RCLCPP_INFO((this->get_logger()), "Read mujoco info node is started!");
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "mjc_exo_state", 10, std::bind(&ReadMujocoInfo::topic_callback, this, _1));
}

void ReadMujocoInfo::topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::vector<MjcStateInfo> msg_info;
    for (int i = 0; i < static_cast<int>(msg->position.size()); i++) {
        MjcStateInfo info;
        info.name = msg->name[i];
        info.mjc_position = msg->position[i];
        info.mjc_velocity = msg->velocity[i];
        info.mjc_effort = msg->effort[i];
        msg_info.push_back(info);
    }
    this->mjc_state_info_ = &msg_info;
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReadMujocoInfo>());
    rclcpp::shutdown();
    return 0;
}