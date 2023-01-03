#include "march_shared_msgs/srv/gait_command.hpp"
#include "mujoco_interfaces/msg/ipd_input.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <chrono>
#include <cstdio>
#include <string>
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class GaitLoader : public rclcpp::Node {
public:
    GaitLoader()
        : Node("gait_loader_node")
    {
        service = this->create_service<march_shared_msgs::srv::GaitCommand>(
            "gait_command", std::bind(&GaitLoader::publish_gait_command, this, _1, _2));
        m_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("gait", 10);
        // For now the joint names are hard-coded, because it is unsure how they will be retrieved in the future.
        m_joint_names = { "Joint1", "joint2", "joint3", "joint4", "joint5", "join6", "joint7", "joint8" };
    };

private:
    void publish_gait_command(const std::shared_ptr<march_shared_msgs::srv::GaitCommand::Request> request,
        std::shared_ptr<march_shared_msgs::srv::GaitCommand::Response> response)
    {
        int gait_type = request->gait_type;
        publish_gait_msg(gait_type);
        response->success = true;
    };

    void publish_gait_msg(const int gait_type)
    {
        // TODO: Select gait based on gait command type, and enum for the gait_commands should be created
        auto message = trajectory_msgs::msg::JointTrajectory();
        message.header.stamp = this->get_clock()->now();
        for (const std::string& joint_name : m_joint_names) {
            message.joint_names.push_back(joint_name);
        }
        // publish the joint points here!!!!w
        auto trajectory_point = trajectory_msgs::msg::JointTrajectoryPoint();

        m_publisher->publish(message);
    };

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr m_publisher;
    rclcpp::Service<march_shared_msgs::srv::GaitCommand>::SharedPtr service;
    std::array<std::string, 8> m_joint_names;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GaitLoader>());

    rclcpp::shutdown();
    return 0;
}