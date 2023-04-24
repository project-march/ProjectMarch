#include "torque_converter/torque_converter_node.hpp"

TorqueConverterNode::TorqueConverterNode()
    : Node("torque_converter")
    , m_torque_converter()
{
    declare_parameter("robot_description", std::string(""));
    auto robot_description = this->get_parameter("robot_description").as_string();
    m_torque_converter.load_urdf_model(robot_description);
     // create subscriber
    m_trajectory_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
        "/torque_trajectory", 10, std::bind(&TorqueConverter::trajectory_subscriber_callback, this, _1));

}

void trajectory_subscriber_callback(sensor_msgs::msg::JointState::SharedPtr msg)
{

}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TorqueConverterNode>());
    rclcpp::shutdown();
    return 0;
}
