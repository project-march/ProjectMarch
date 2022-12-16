#include <cstdio>
#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "mujoco_interfaces/msg/ipd_input.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;

class GaitLoader:public rclcpp::Node {
public:
    GaitLoader()
        : Node("gait_loader_node"){
        m_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("gait", 10);
        m_subscriber = this->create_subscription<mujoco_interfaces::msg::IpdInput>("ipd_command", 10,
            std::bind(&GaitLoader::subscriber_callback, this, _1));
        m_joint_names = {"Joint1", "joint2", "joint3", "joint4", "joint5", "join6", "joint7", "joint8"};

        m_timer = this->create_wall_timer(
            1000ms, std::bind(&GaitLoader::timer_callback, this));
    };

private:
    void
    subscriber_callback(mujoco_interfaces::msg::IpdInput::SharedPtr msg) {
    };

    void timer_callback(){
        publish_gait_msg();
    };

    void publish_gait_msg() {
        auto message = trajectory_msgs::msg::JointTrajectory();
        message.header.stamp = this->get_clock()->now();
        for(const std::string &joint_name : m_joint_names){
            message.joint_names.push_back(joint_name);
        }
        //publish the joint points here!!!!w
        auto trajectory_point = trajectory_msgs::msg::JointTrajectoryPoint();

        m_publisher->publish(message);
    };


    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr m_publisher;
    rclcpp::Subscription<mujoco_interfaces::msg::IpdInput>::SharedPtr m_subscriber;
    std::array<std::string, 8> m_joint_names;

    rclcpp::TimerBase::SharedPtr m_timer;

};

int main(int argc, char ** argv)
{

    printf("hello world gait_loader package\n");

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GaitLoader>());
    rclcpp::shutdown();
    return 0;
}