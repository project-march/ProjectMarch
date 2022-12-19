#include <cstdio>
#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "march_shared_msgs/msg/robot_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
using std::placeholders::_1;

class StateEstimator:public rclcpp::Node {
public:
    StateEstimator()
        : Node("state_estimator_node"){
        state_publisher = this->create_publisher<march_shared_msgs::msg::RobotState>("robot_state", 10);
        sensor_subscriber = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 10,
            std::bind(&StateEstimator::sensor_callback, this, _1));
        state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>("/joint_state", 10,
            std::bind(&StateEstimator::state_callback, this, _1));
    };
private:
    void sensor_callback(sensor_msgs::msg::Imu msg){
    };

    void state_callback(sensor_msgs::msg::JointState msg){
    };

    void publish_robot_state(){
        auto msg = march_shared_msgs::msg::RobotState();
        message.header.stamp = this->get_clock()->now();
        msg.joint_names.push_back("");
        msg.joint_pos.push_back(0);
        msg.joint_vel.push_back(0);
        msg.sensor_name.push_back("");
        msg.sensor_data.push_back(0);

        state_publisher->publish(msg)
    };

    rclcpp::Publisher<march_shared_msgs::msg::RobotState>::SharedPtr state_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sensor_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscriber;

};

int main(int argc, char ** argv)
{
    printf("state_estimator node started!\n");

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GaitLoader>());
    rclcpp::shutdown();
    return 0;
}
