//
// Created by Marco Bak march8 on 1-2-23.
//
#include "swing_leg_trajectory_generator/swing_leg_trajectory_generator_node.hpp"
#include "swing_leg_trajectory_generator/swing_leg_trajectory_generator.hpp"
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

SwingLegTrajectoryGeneratorNode::SwingLegTrajectoryGeneratorNode()
    : Node("swing_leg_trajectory_generator_node")
    , prev_step_size(0.0)
{
    m_publish_curve = this->create_publisher<geometry_msgs::msg::PoseArray>("bezier_trajectory", 10);
    m_points_subscription = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "bezier_points", 10, std::bind(&SwingLegTrajectoryGeneratorNode::subscriber_callback, this, _1));

    m_weight_shift_subscriber = this->create_subscription<std_msgs::msg::Int32>(
        "publish_swing_leg_command", 10, std::bind(&SwingLegTrajectoryGeneratorNode::weight_shift_callback, this, _1));

    m_final_feet_subscriber = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "final_feet_position", 10, std::bind(&SwingLegTrajectoryGeneratorNode::final_feet_callback, this, _1));

    //    m_stance_feet_subscriber = this->create_subscription<std_msgs::msg::Int32>(
    //        "current_stance_foot", 10, std::bind(&SwingLegTrajectoryGeneratorNode::stance_feet_callback, this, _1));
    m_swing_leg_generator = SwingLegTrajectoryGenerator();

    m_path_publisher = this->create_publisher<nav_msgs::msg::Path>("path_visualization", 10);
}

void SwingLegTrajectoryGeneratorNode::weight_shift_callback(std_msgs::msg::Int32::SharedPtr msg)
{
    publish_zero_swing();
}

void SwingLegTrajectoryGeneratorNode::subscriber_callback(geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    std::vector<Point> points;
    points.reserve(msg->poses.size());
    for (auto& p : msg->poses) {
        points.push_back(p.position);
    }
    m_swing_leg_generator.set_points(points);
}

void SwingLegTrajectoryGeneratorNode::publish_zero_swing()
{
    geometry_msgs::msg::PoseArray empty_poses;
    geometry_msgs::msg::Pose empty_pose;
    empty_pose.position.x = 0.0;
    empty_pose.position.y = 0.0;
    empty_pose.position.z = 0.0;
    empty_poses.poses.push_back(empty_pose);
    empty_poses.poses.push_back(empty_pose);
    m_publish_curve->publish(empty_poses);
}

// void SwingLegTrajectoryGeneratorNode::stance_feet_callback(std_msgs::msg::Int32::SharedPtr msg)
//{
//    m_swing_leg_generator.generate_trajectory();
//    m_publish_curve->publish(m_swing_leg_generator.get_curve().trajectory);
//}

void SwingLegTrajectoryGeneratorNode::final_feet_callback(geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    auto steps = msg->poses;
    auto begin_foot = steps.at(0);
    auto end_foot = steps.at(2);
    double step_size = prev_step_size + steps.at(0).position.x;
    if (step_size <= 0.00001) {
        return;
    }
    m_swing_leg_generator.set_step_length(step_size);
    // RCLCPP_WARN(this->get_logger(), "Step size is %f \n\n\n\n\n", step_size);
    m_publish_curve->publish(m_swing_leg_generator.get_curve().trajectory);
    prev_step_size = steps.at(0).position.x;
    publish_path_visualization();
}

void SwingLegTrajectoryGeneratorNode::publish_path_visualization()
{
    nav_msgs::msg::Path msg;
    msg.header.frame_id = "map";
    msg.header.stamp = this->get_clock()->now();

    geometry_msgs::msg::PoseStamped pose_container;
    for (int i = 0; i < m_swing_leg_generator.get_curve().trajectory.poses.size(); i++) {
        pose_container.pose = m_swing_leg_generator.get_curve().trajectory.poses[i];
        pose_container.header.frame_id = "map";
        pose_container.header.stamp = msg.header.stamp;
        pose_container.header.stamp.nanosec += 1e9 * i;
        msg.poses.push_back(pose_container);
    }
    m_path_publisher->publish(msg);
}

/**
 * Main function to run the node.
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwingLegTrajectoryGeneratorNode>());

    rclcpp::shutdown();
    return 0;
}