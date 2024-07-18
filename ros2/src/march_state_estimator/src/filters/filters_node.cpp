/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_state_estimator/filters/filter_nodes.hpp"

#include <cmath>
#include <functional>
#include <boost/algorithm/clamp.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "urdf/model.h"

FiltersNode::FiltersNode() 
    : Node("filters_node")
{
    // Configure the mean filters for the linear acceleration, and angular velocity 
    for (unsigned i = 0; i < NUM_CHANNELS_IMU_ACC; i++) {
        // Create mean filters for the linear acceleration and angular velocity
        MyMeanFilter::SharedPtr acc_mean_filter = std::make_shared<MyMeanFilter>();
        acc_mean_filter->configure(m_imu_acc_window_size);
        m_imu_acc_mean_filters.push_back(acc_mean_filter);

        MyMeanFilter::SharedPtr gyro_mean_filter = std::make_shared<MyMeanFilter>();
        gyro_mean_filter->configure(m_imu_gyro_window_size);
        m_imu_gyro_mean_filters.push_back(gyro_mean_filter);
    }

    // Configure the mean filters for the torque values
    for (unsigned i = 0; i < NUM_CHANNELS_TORQUE; i++) {
        MyMeanFilter::SharedPtr torque_mean_filter = std::make_shared<MyMeanFilter>();
        torque_mean_filter->configure(m_torque_window_size);
        m_torque_mean_filters.push_back(torque_mean_filter);
    }

    // Parse URDF file to get joint names, and the corresponding joint limits
    declare_parameter("urdf_file_path", "");
    std::string urdf_file = this->get_parameter("urdf_file_path").as_string();

    urdf::Model model;
    if (!model.initFile(urdf_file)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF file.");
        return;
    }

    // Get the joint names, and the corresponding joint limits, and initialize the joint position, velocity, and effort maps
    for (auto joint : model.joints_) {
        if (joint.second->type != urdf::Joint::REVOLUTE) {
            continue;
        }
        m_joint_position_map[joint.first] = 0.0;
        m_joint_velocity_map[joint.first] = 0.0;
        m_joint_effort_map[joint.first] = 0.0;
        m_joint_position_limits_map[joint.first][LOWER_LIMIT] = joint.second->limits->lower;
        m_joint_position_limits_map[joint.first][UPPER_LIMIT] = joint.second->limits->upper;
    }

    // Get the joint names that are blacklisted
    // TODO: Parameterize the blacklisted joint names
    m_blacklist_joint_names = {
        "left_hip_aa", "left_hip_fe", "left_knee", "left_ankle_ie", 
        "right_hip_aa", "right_hip_fe", "right_knee", "right_ankle_ie"
    };

    // Create the subscriptions and publishers
    m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        "lower_imu", rclcpp::SensorDataQoS(), std::bind(&FiltersNode::imuCallback, this, std::placeholders::_1));
    m_imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("lower_imu/filtered", 10);

    m_joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::SensorDataQoS(), std::bind(&FiltersNode::jointStateCallback, this, std::placeholders::_1));
    m_joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states/filtered", 10);

    RCLCPP_INFO(this->get_logger(), "Filters node has been started.");
}

void FiltersNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    sensor_msgs::msg::Imu filtered_msg = *msg;

    // Filter the linear acceleration and angular velocity
    m_imu_acc_mean_filters[AXIS_X]->update(msg->linear_acceleration.x, filtered_msg.linear_acceleration.x);
    m_imu_acc_mean_filters[AXIS_Y]->update(msg->linear_acceleration.y, filtered_msg.linear_acceleration.y);
    m_imu_acc_mean_filters[AXIS_Z]->update(msg->linear_acceleration.z, filtered_msg.linear_acceleration.z);

    m_imu_gyro_mean_filters[AXIS_X]->update(msg->angular_velocity.x, filtered_msg.angular_velocity.x);
    m_imu_gyro_mean_filters[AXIS_Y]->update(msg->angular_velocity.y, filtered_msg.angular_velocity.y);
    m_imu_gyro_mean_filters[AXIS_Z]->update(msg->angular_velocity.z, filtered_msg.angular_velocity.z);

    m_imu_pub->publish(filtered_msg);
}

void FiltersNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    sensor_msgs::msg::JointState filtered_msg = *msg;

    // Check if the size of the joint state message is correct
    if (msg->name.size() != msg->position.size() || msg->name.size() != msg->velocity.size() || msg->name.size() != msg->effort.size()) {
        RCLCPP_ERROR(this->get_logger(), "The size of the received joint state message is incorrect.");
        return;
    }

    // Filter the joint positions, velocities, and efforts if they are NaN
    for (unsigned i = 0; i < msg->name.size(); i++) {
        if (!std::isnan(msg->position[i])) {
            m_joint_position_map[msg->name[i]] = boost::algorithm::clamp(
                msg->position[i], m_joint_position_limits_map[msg->name[i]][LOWER_LIMIT], m_joint_position_limits_map[msg->name[i]][UPPER_LIMIT]);
        }
        if (!std::isnan(msg->velocity[i])) {
            m_joint_velocity_map[msg->name[i]] = msg->velocity[i];
        }
        if (!std::isnan(msg->effort[i])) {
            m_joint_effort_map[msg->name[i]] = msg->effort[i];
        }
    }

    // Filter the joint efforts
    for (unsigned i = 0; i < NUM_CHANNELS_TORQUE; i++) {
        // Filter the torque values
        m_torque_mean_filters[i]->update(m_joint_effort_map[msg->name[i]], filtered_msg.effort[i]);
    }

    // Replace the joint positions, and velocities with the filtered values
    for (unsigned i = 0; i < msg->name.size(); i++) {
        filtered_msg.position[i] = m_joint_position_map[msg->name[i]];
        filtered_msg.velocity[i] = m_joint_velocity_map[msg->name[i]];

        // Replace the joint velocity and effort to zero if joint is blacklisted
        if (jointNameBlacklisted(msg->name[i])) {
            filtered_msg.velocity[i] = 0.0;
            filtered_msg.effort[i] = 0.0;
        }
    }

    m_joint_state_pub->publish(filtered_msg);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FiltersNode>());
    rclcpp::shutdown();
    return 0;
}