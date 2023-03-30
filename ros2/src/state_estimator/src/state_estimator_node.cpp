#include "geometry_msgs/msg/point_stamped.hpp"
#include "march_shared_msgs/msg/robot_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <chrono>
#include <cstdio>
#include <string>

using std::placeholders::_1;

class StateEstimator : public rclcpp::Node {
public:
    StateEstimator()
        : Node("state_estimator_node")
    {
        //        state_publisher = this->create_publisher<march_shared_msgs::msg::RobotState>("robot_state", 10);
        com_publisher = this->create_publisher<geometry_msgs::msg::PointStamped>("robot_com_position", 10);
        zmp_publisher = this->create_publisher<geometry_msgs::msg::PointStamped>("robot_zmp_position", 10);
        foot_pos_publisher = this->create_publisher<geometry_msgs::msg::PointStamped>("est_foot_position", 10);
        joint_state_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_state", 10);
        imu_state_publisher = this->create_publisher<sensor_msgs::msg::Imu>("imu_state", 10);

        //        sensor_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
        //            "/imu", 10, std::bind(&StateEstimator::sensor_callback, this, _1));
        //        state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
        //            "/joint_state", 10, std::bind(&StateEstimator::state_callback, this, _1));
    };

private:
    void sensor_callback(sensor_msgs::msg::Imu::SharedPtr msg) {};

    void state_callback(sensor_msgs::msg::JointState::SharedPtr msg) {};

    void publish_robot_state()
    {
        auto msg = march_shared_msgs::msg::RobotState();
        msg.stamp = this->get_clock()->now();
        msg.joint_names.push_back("");
        msg.joint_pos.push_back(0);
        msg.joint_vel.push_back(0);
        msg.sensor_names.push_back("");
        msg.sensor_data.push_back(0);

        state_publisher->publish(msg);
    };

    rclcpp::Publisher<march_shared_msgs::msg::RobotState>::SharedPtr state_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sensor_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscriber;

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr com_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr zmp_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr foot_pos_publisher;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_state_publisher;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateEstimator>());
    rclcpp::shutdown();
    return 0;
}
