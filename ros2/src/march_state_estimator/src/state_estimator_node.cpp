#include "march_state_estimator/state_estimator_node.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <chrono>
#include <memory>
#include <functional>
#include <future>

using std::placeholders::_1;
using std::placeholders::_2;

StateEstimatorNode::StateEstimatorNode()
    : Node("state_estimator_node")
{
    // Declare the parameters
    this->declare_parameter<int64_t>("dt", 50);

    // Get the parameters
    int64_t dt = this->get_parameter("dt").as_int();
    m_dt = static_cast<double>(dt) / 1000.0;

    // Initialize the node
    m_joint_state_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_imu_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_timer_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_node_position_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_joint_state_subscription_options.callback_group = m_joint_state_callback_group;
    m_imu_subscription_options.callback_group = m_imu_callback_group;

    m_timer = this->create_wall_timer(std::chrono::milliseconds(dt), std::bind(&StateEstimatorNode::timerCallback, this), m_timer_callback_group);
    m_joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::SensorDataQoS(), std::bind(&StateEstimatorNode::jointStateCallback, this, std::placeholders::_1), m_joint_state_subscription_options);
    m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu", rclcpp::SensorDataQoS(), std::bind(&StateEstimatorNode::imuCallback, this, std::placeholders::_1), m_imu_subscription_options);
    m_state_estimation_pub = this->create_publisher<march_shared_msgs::msg::StateEstimation>("state_estimation/state", 1);
    m_get_node_position_client = this->create_client<march_shared_msgs::srv::GetNodePosition>("state_estimation/get_node_position", 
        rmw_qos_profile_services_default, m_node_position_callback_group);

    // Set the initial joint state message to zero data
    sensor_msgs::msg::JointState init_jointstate_msg;
    init_jointstate_msg.header.stamp = this->now();
    init_jointstate_msg.header.frame_id = "backpack";
    init_jointstate_msg.name = {};
    init_jointstate_msg.position = {};
    init_jointstate_msg.velocity = {};
    init_jointstate_msg.effort = {};

    m_joint_state = std::make_shared<sensor_msgs::msg::JointState>(init_jointstate_msg);

    sensor_msgs::msg::Imu init_imu_msg;
    // Set the initial imu message to zero data
    init_imu_msg.header.stamp = this->now();
    init_imu_msg.header.frame_id = "backpack";
    init_imu_msg.orientation.x = 0.0;
    init_imu_msg.orientation.y = 0.0;
    init_imu_msg.orientation.z = 0.0;
    init_imu_msg.orientation.w = 1.0;
    init_imu_msg.angular_velocity.x = 0.0;
    init_imu_msg.angular_velocity.y = 0.0;
    init_imu_msg.angular_velocity.z = 0.0;
    init_imu_msg.linear_acceleration.x = 0.0;
    init_imu_msg.linear_acceleration.y = 0.0;
    init_imu_msg.linear_acceleration.z = 0.0;

    m_imu = std::make_shared<sensor_msgs::msg::Imu>(init_imu_msg);

    m_node_feet_names = {"L_foot", "R_foot"};

    RCLCPP_INFO(this->get_logger(), "State Estimator Node initialized");
}

void StateEstimatorNode::timerCallback()
{
    // Publish the state estimation
    publishStateEstimation();
}

void StateEstimatorNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Store the joint state message
    m_joint_state = msg;

    // Request the node positions
    requestNodePositions(msg);
}

void StateEstimatorNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // Store the imu message
    m_imu = msg;
}

void StateEstimatorNode::nodePositionCallback(
    const rclcpp::Client<march_shared_msgs::srv::GetNodePosition>::SharedFuture future)
{
    RCLCPP_DEBUG(this->get_logger(), "Received node positions");

    // Get the response
    march_shared_msgs::srv::GetNodePosition::Response::SharedPtr response_msg = future.get();

    // Store the node positions
    m_foot_positions = response_msg->node_positions;

    // Print the node positions
    for (auto node_position : m_foot_positions)
    {
        RCLCPP_DEBUG(this->get_logger(), "Node position: %f, %f, %f", node_position.x, node_position.y, node_position.z);
    }
}

void StateEstimatorNode::requestNodePositions(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Create a request message
    march_shared_msgs::srv::GetNodePosition::Request::SharedPtr request =
        std::make_shared<march_shared_msgs::srv::GetNodePosition::Request>();

    // Fill the request message with data
    request->node_names = m_node_feet_names;
    request->joint_names = msg->name;
    request->joint_positions = msg->position;

    // Send the request
    m_get_node_position_future = m_get_node_position_client->async_send_request(request,
        std::bind(&StateEstimatorNode::nodePositionCallback, this, std::placeholders::_1));

    // // Wait for the response
    // if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), m_get_node_position_future) !=
    //     rclcpp::FutureReturnCode::SUCCESS)
    // {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to get node positions");
    //     return;
    // }
}

void StateEstimatorNode::publishStateEstimation()
{
    // Create a state estimation message
    march_shared_msgs::msg::StateEstimation state_estimation_msg;

    // Convert the node positions to poses
    std::vector<geometry_msgs::msg::Pose> foot_poses;
    uint8_t stance_leg = 0;
    for (long unsigned int i = 0; i < m_foot_positions.size(); i++)
    {
        RCLCPP_DEBUG(this->get_logger(), "Foot position: %f, %f, %f", m_foot_positions[i].x, m_foot_positions[i].y, m_foot_positions[i].z);
        geometry_msgs::msg::Pose foot_pose;
        foot_pose.position = m_foot_positions[i];
        foot_pose.orientation.x = 0.0;
        foot_pose.orientation.y = 0.0;
        foot_pose.orientation.z = 0.0;
        foot_pose.orientation.w = 1.0;
        foot_poses.push_back(foot_pose);

        // Check if the foot is on the ground. TODO: This should be done in a better way *cough* contact detection *cough*
        // TODO: Fix this when stance leg is in front of the robot
        if (m_foot_positions[i].x < 0.265)
        {
            RCLCPP_DEBUG(this->get_logger(), "%s is on the ground", m_node_feet_names[i].c_str());
            stance_leg = stance_leg | (0b1 << i);
        }
        RCLCPP_DEBUG(this->get_logger(), "Stance leg: %hu", stance_leg);
    }

    // Fill the message with data
    state_estimation_msg.header.stamp = this->now();
    state_estimation_msg.header.frame_id = "backpack";
    state_estimation_msg.step_time = m_dt;
    state_estimation_msg.joint_state = *m_joint_state;
    state_estimation_msg.imu = *m_imu;
    state_estimation_msg.foot_pose = foot_poses;
    state_estimation_msg.stance_leg = stance_leg;

    // Publish the message
    m_state_estimation_pub->publish(state_estimation_msg);
}

void StateEstimatorNode::handleGetCurrentJointPositions(std::shared_ptr<march_shared_msgs::srv::GetCurrentJointPositions::Request>,
    std::shared_ptr<march_shared_msgs::srv::GetCurrentJointPositions::Response> response)
{
    // Fill the response message with data
    response->joint_positions = m_joint_state->position;
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<StateEstimatorNode>());
    auto node = std::make_shared<StateEstimatorNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}