#include "march_ik_solver/ik_solver_buffer_node.hpp"

IKSolverBufferNode::IKSolverBufferNode()
    : Node("ik_solver_buffer", rclcpp::NodeOptions())
{
    // Declare the node parameters.
    declare_parameter("dt", 0.01);
    declare_parameter("joints_size", 8);
    declare_parameter("convergence_threshold", 0.001);
    declare_parameter("early_stopping", false);
    declare_parameter("early_stopping_threshold", 5);

    // Initialize the node parameters.
    n_joints_ = get_parameter("joints_size").as_int();
    dt_ = get_parameter("dt").as_double();
    convergence_threshold_ = get_parameter("convergence_threshold").as_double();
    early_stopping_ = get_parameter("early_stopping").as_bool();
    early_stopping_threshold_ = get_parameter("early_stopping_threshold").as_int();
    RCLCPP_INFO(this->get_logger(), "dt: %f", dt_);
    RCLCPP_INFO(this->get_logger(), "convergence_threshold: %f", convergence_threshold_);

    // Create a timer that will publish the joint trajectory buffer.
    auto timer_callback = std::bind(&IKSolverBufferNode::publishIKSolverFootPositions, this);
    timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(dt_ * 1000)), timer_callback);

    // Create a subscription to the joint states.
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&IKSolverBufferNode::jointStateCallback, this, std::placeholders::_1));

    // Create a subscription to the joint trajectory.
    joint_trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "joint_trajectory_controller/joint_trajectory", 10, std::bind(&IKSolverBufferNode::jointTrajectoryCallback, this, std::placeholders::_1));

    // Create a subscription to the Exo state.
    exo_state_sub_ = this->create_subscription<march_shared_msgs::msg::ExoState>(
        "current_state", 10, std::bind(&IKSolverBufferNode::exoStateCallback, this, std::placeholders::_1));

    // Create a subscription to the IK solver foot positions.
    ik_solver_foot_positions_sub_ = this->create_subscription<march_shared_msgs::msg::IksFootPositions>(
        "ik_solver/buffer/input", 10, std::bind(&IKSolverBufferNode::ikSolverFootPositionsCallback, this, std::placeholders::_1));

    // Create a publisher for the IK solver foot positions.
    ik_solver_foot_positions_pub_ = this->create_publisher<march_shared_msgs::msg::IksFootPositions>(
        "ik_solver/buffer/output", 10);

    // Create a publisher for the IK solver status.
    ik_solver_status_pub_ = this->create_publisher<std_msgs::msg::UInt32>(
        "ik_solver/status", 1);
    ik_solver_error_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "ik_solver/error", 1);

    // Set the gait type to -1 as None, and set the gait reset to passthrough the first IK solver foot positions.
    gait_type_ = -1;
    gait_reset_ = true;
    ik_solver_foot_positions_received_ = false;
}

void IKSolverBufferNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Store the current joint positions.
    // current_joint_positions_ = Eigen::Map<Eigen::VectorXd>(msg->position.data(), n_joints_);
    Eigen::VectorXd current_joint_positions = Eigen::VectorXd::Zero(n_joints_);
    current_joint_positions(0) = msg->position[3];
    current_joint_positions(1) = msg->position[0];
    current_joint_positions(2) = msg->position[1];
    current_joint_positions(3) = msg->position[2];
    current_joint_positions(4) = msg->position[7];
    current_joint_positions(5) = msg->position[4];
    current_joint_positions(6) = msg->position[5];
    current_joint_positions(7) = msg->position[6];
    current_joint_positions_ = current_joint_positions;
}

void IKSolverBufferNode::jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
    // Store the joint trajectory in the buffer.
    desired_joint_positions_ = Eigen::Map<Eigen::VectorXd>(msg->points.back().positions.data(), n_joints_);
}

void IKSolverBufferNode::exoStateCallback(const march_shared_msgs::msg::ExoState::SharedPtr msg)
{
    // Store the gait type.
    gait_type_ = msg->state;

    // Reset the gait reset flag.
    gait_reset_ = true;

    // Clear the buffer.
    ik_solver_foot_positions_buffer_.clear();
}

void IKSolverBufferNode::ikSolverFootPositionsCallback(const march_shared_msgs::msg::IksFootPositions::SharedPtr msg)
{
    // Store the IK solver foot positions in the buffer.
    // ik_solver_foot_positions_buffer_.push_back(*msg);
    ik_solver_foot_positions_latest_ = *msg;
    ik_solver_foot_positions_received_ = true;

    // Send the first IK solver foot positions to the IK solver.
    march_shared_msgs::msg::IksFootPositions msg_out;
    msg_out.header.stamp = this->now();
    msg_out.left_foot_position.x = msg->left_foot_position.x;
    msg_out.left_foot_position.y = msg->left_foot_position.y;
    msg_out.left_foot_position.z = msg->left_foot_position.z;
    msg_out.right_foot_position.x = msg->right_foot_position.x;
    msg_out.right_foot_position.y = msg->right_foot_position.y;
    msg_out.right_foot_position.z = msg->right_foot_position.z;
    msg_out.time_from_start.sec = 0;
    msg_out.time_from_start.nanosec = (uint32_t) (dt_ * 1e9);
    msg_out.new_point = true;
    ik_solver_foot_positions_pub_->publish(msg_out);
}

void IKSolverBufferNode::publishIKSolverFootPositions()
{
    // Check if the gait type is set.
    if (gait_type_ == -1 || !ik_solver_foot_positions_received_)
    {
        return;
    }

    // // Check if the buffer is empty.
    // if (ik_solver_foot_positions_buffer_.size() > 0)
    // {
    //     // Check if the gait reset flag is set.
    //     if (gait_reset_)
    //     {
    //         // Reset the gait reset flag.
    //         gait_reset_ = false;

    //         // // Set the current joint positions to the first IK solver foot positions.
    //         // current_joint_positions_ = Eigen::Map<Eigen::VectorXd>(ik_solver_foot_positions_buffer_.front().positions.data(), n_joints_);

    //         // Publish the IK solver foot positions.
    //         march_shared_msgs::msg::IksFootPositions msg;
    //         msg.header.stamp = this->now();
    //         msg.left_foot_position.x = ik_solver_foot_positions_buffer_.front().left_foot_position.x;
    //         msg.left_foot_position.y = ik_solver_foot_positions_buffer_.front().left_foot_position.y;
    //         msg.left_foot_position.z = ik_solver_foot_positions_buffer_.front().left_foot_position.z;
    //         msg.right_foot_position.x = ik_solver_foot_positions_buffer_.front().right_foot_position.x;
    //         msg.right_foot_position.y = ik_solver_foot_positions_buffer_.front().right_foot_position.y;
    //         msg.right_foot_position.z = ik_solver_foot_positions_buffer_.front().right_foot_position.z; 
    //         msg.time_from_start.sec = 0;
    //         msg.time_from_start.nanosec = 50;
    //         ik_solver_foot_positions_pub_->publish(msg);

    //         // Clear the buffer.
    //         ik_solver_foot_positions_buffer_.clear();
    //     }
    //     else
    //     {
    //         // Calculate the joint position error.
    //         double error = (desired_joint_positions_ - current_joint_positions_).norm();

    //         // Publish the IK solver error.
    //         publishIKSolverError(error);

    //         // Check if the desired joint positions are reached.
    //         // RCLCPP_INFO(this->get_logger(), "Error: %f", error);
    //         // RCLCPP_INFO(this->get_logger(), "Convergence threshold: %f", convergence_threshold_);
    //         if (early_stopping_ || (ik_solver_foot_positions_buffer_.size() > early_stopping_threshold_))
    //         {
    //             march_shared_msgs::msg::IksFootPositions msg;
    //             msg.header.stamp = this->now();
    //             msg.left_foot_position.x = ik_solver_foot_positions_buffer_.back().left_foot_position.x;
    //             msg.left_foot_position.y = ik_solver_foot_positions_buffer_.back().left_foot_position.y;
    //             msg.left_foot_position.z = ik_solver_foot_positions_buffer_.back().left_foot_position.z;
    //             msg.right_foot_position.x = ik_solver_foot_positions_buffer_.back().right_foot_position.x;
    //             msg.right_foot_position.y = ik_solver_foot_positions_buffer_.back().right_foot_position.y;
    //             msg.right_foot_position.z = ik_solver_foot_positions_buffer_.back().right_foot_position.z; 
    //             msg.time_from_start.sec = 0;
    //             msg.time_from_start.nanosec = (uint32_t) (dt_ * 1e9);
    //             ik_solver_foot_positions_pub_->publish(msg);

    //             // Clear the buffer.
    //             ik_solver_foot_positions_buffer_.clear();
    //             ik_solver_foot_potion.x = ik_solver_foot_positions_buffer_.front().left_foot_position.x;
    //             msg.left_foot_position.y = ik_solver_foot_positions_buffer_.front().left_foot_position.y;
    //             msg.left_foot_position.z = ik_solver_foot_positions_buffer_.front().left_foot_position.z;
    //             msg.right_foot_position.x = ik_solver_foot_positions_buffer_.front().right_foot_position.x;
    //             msg.right_foot_position.y = ik_solver_foot_positions_buffer_.front().right_foot_position.y;
    //             msg.right_foot_position.z = ik_solver_foot_positions_buffer_.front().right_foot_position.z; 
    //             msg.time_from_start.sec = 0;
    //             msg.time_from_start.nanosec = (uint32_t) (dt_ * 1e9);
    //             ik_solver_foot_positions_pub_->publish(msg);
    //             ik_solver_foot_positions_latest_ = msg;
    //             // RCLCPP_INFO(this->get_logger(), "New front: %f, %f, %f, %f, %f, %f", 
    //             //     msg.left_foot_position.x, msg.left_foot_position.y, msg.left_foot_position.z, 
    //             //     msg.right_foot_position.x, msg.right_foot_position.y, msg.right_foot_position.z);
    //         }
    //         else
    //         {
    //             // Publish the IK solver foot positions.
    //             march_shared_msgs::msg::IksFootPositions msg;
    //             msg.header.stamp = this->now();
    //             msg.left_foot_position.x = ik_solver_foot_positions_buffer_.front().left_foot_position.x;
    //             msg.left_foot_position.y =itions msg;
    //     msg.header.stamp = this->now();
    //     msg.left_foot_position.x = ik_solver_foot_positions_latest_.left_foot_position.x;
    //     msg.left_foot_position.y = ik_solver_foot_positions_latest_.left_foot_position.y;
    //     msg.left_foot_position.z = ik_solver_foot_positions_latest_.left_foot_position.z;
    //     msg.right_foot_position.x = ik_solver_foot_positions_latest_.right_foot_position.x;
    //     msg.right_foot_positiontion.x = ik_solver_foot_positions_buffer_.front().left_foot_position.x;
    //             msg.left_foot_position.y = ik_solver_foot_positions_buffer_.front().left_foot_position.y;
    //             msg.left_foot_position.z = ik_solver_foot_positions_buffer_.front().left_foot_position.z;
    //             msg.right_foot_position.x = ik_solver_foot_positions_buffer_.front().right_foot_position.x;
    //             msg.right_foot_position.y = ik_solver_foot_positions_buffer_.front().right_foot_position.y;
    //             msg.right_foot_position.z = ik_solver_foot_positions_buffer_.front().right_foot_position.z; 
    //             msg.time_from_start.sec = 0;
    //             msg.time_from_start.nanosec = (uint32_t) (dt_ * 1e9);
    //             ik_solver_foot_positions_pub_->publish(msg);
    //             ik_solver_foot_positions_latest_ = msg;
    //             // RCLCPP_INFO(this->get_logger(), "New front: %f, %f, %f, %f, %f, %f", 
    //             //     msg.left_foot_position.x, msg.left_foot_position.y, msg.left_foot_position.z, 
    //             //     msg.right_foot_position.x, msg.right_foot_position.y, msg.right_foot_position.z);
    //         }
    //         else
    //         {
    //             // Publish the IK solver foot positions.
    //             march_shared_msgs::msg::IksFootPositions msg;
    //             msg.header.stamp = this->now();
    //             msg.left_foot_position.x = ik_solver_foot_positions_buffer_.front().left_foot_position.x;
    //             msg.left_foot_position.y =itions msg;
    //     msg.header.stamp = this->now();
    //     msg.left_foot_position.x = ik_solver_foot_positions_latest_.left_foot_position.x;
    //     msg.left_foot_position.y = ik_solver_foot_positions_latest_.left_foot_position.y;
    //     msg.left_foot_position.z = ik_solver_foot_positions_latest_.left_foot_position.z;
    //     msg.right_foot_position.x = ik_solver_foot_positions_latest_.right_foot_position.x;
    //     msg.right_foot_position.y = ik_solver_foot_positions_latest_.right_foot_position.y;
    //     msg.right_foot_position.z = ik_solver_foot_positions_latest_.right_foot_position.z;
    //     msg.time_from_start.sec = 0;
    //             ik_solver_foot_positions_pub_->publish(msg);
    //             ik_solver_foot_positions_latest_ = msg;
    //             // RCLCPP_INFO(this->get_logger(), "Old front: %f, %f, %f, %f, %f, %f", 
    //             //     msg.left_foot_position.x, msg.left_foot_position.y, msg.left_foot_position.z, 
    //             //     msg.right_foot_position.x, msg.right_foot_position.y, msg.right_foot_position.z);
    //         }
    //     }
    // }
    // else
    // {
    //     // Publish the IK solver foot positions.
    //     march_shared_msgs::msg::IksFootPositions msg;
    //     msg.header.stamp = this->now();
    //     msg.left_foot_position.x = ik_solver_foot_positions_latest_.left_foot_position.x;
    //     msg.left_foot_position.y = ik_solver_foot_positions_latest_.left_foot_position.y;
    //     msg.left_foot_position.z = ik_solver_foot_positions_latest_.left_foot_position.z;
    //     msg.right_foot_position.x = ik_solver_foot_positions_latest_.right_foot_position.x;
    //     msg.right_foot_position.y = ik_solver_foot_positions_latest_.right_foot_position.y;
    //     msg.right_foot_position.z = ik_solver_foot_positions_latest_.right_foot_position.z;
    //     msg.time_from_start.sec = 0;
    //     msg.time_from_start.nanosec = (uint32_t) (dt_ * 1e9);
    //     msg.right_foot_position.z = ik_solver_foot_positions_latest_.right_foot_position.z;
    //     msg.time_from_start.sec = 0;
    //             ik_solver_foot_positions_pub_->publish(msg);
    //             ik_solver_foot_positions_latest_ = msg;
    //             // RCLCPP_INFO(this->get_logger(), "Old front: %f, %f, %f, %f, %f, %f", 
    //             //     msg.left_foot_position.x, msg.left_foot_position.y, msg.left_foot_position.z, 
    //             //     msg.right_foot_position.x, msg.right_foot_position.y, msg.right_foot_position.z);
    //         }
    //     }
    // }
    // else
    // {
    //     // Publish the IK solver foot positions.
    //     march_shared_msgs::msg::IksFootPositions msg;
    //     msg.header.stamp = this->now();
    //     msg.left_foot_position.x = ik_solver_foot_positions_latest_.left_foot_position.x;
    //     msg.left_foot_position.y = ik_solver_foot_positions_latest_.left_foot_position.y;
    //     msg.left_foot_position.z = ik_solver_foot_positions_latest_.left_foot_position.z;
    //     msg.right_foot_position.x = ik_solver_foot_positions_latest_.right_foot_position.x;
    //     msg.right_foot_position.y = ik_solver_foot_positions_latest_.right_foot_position.y;
    //     msg.right_foot_position.z = ik_solver_foot_positions_latest_.right_foot_position.z;
    //     msg.time_from_start.sec = 0;
    //     msg.time_from_start.nanosec = (uint32_t) (dt_ * 1e9);
    //     ik_solver_foot_positions_pub_->publish(msg);sitions_latest_ = msg;
    //             // RCLCPP_INFO(this->get_logger(), "Early stopping");
    //             return;
    //         }
    //         if (error < convergence_threshold_)
    //         {
    //             // Remove the first IK solver foot positions from the buffer.
    //             ik_solver_foot_positions_buffer_.erase(ik_solver_foot_positions_buffer_.begin());

    //             march_shared_msgs::msg::IksFootPositions msg;
    //             msg.header.stamp = this->now();
    //             msg.left_foot_position.x = ik_solver_foot_positions_buffer_.front().left_foot_position.x;
    //             msg.left_foot_position.y = ik_solver_foot_positions_buffer_.front().left_foot_position.y;
    //             msg.left_foot_position.z = ik_solver_foot_positions_buffer_.front().left_foot_position.z;
    //             msg.right_foot_position.x = ik_solver_foot_positions_buffer_.front().right_foot_position.x;
    //             msg.right_foot_position.y = ik_solver_foot_positions_buffer_.front().right_foot_position.y;
    //             msg.right_foot_position.z = ik_solver_foot_positions_buffer_.front().right_foot_position.z; 
    //             msg.time_from_start.sec = 0;
    //             msg.time_from_start.nanosec = (uint32_t) (dt_ * 1e9);
    //             ik_solver_foot_positions_pub_->publish(msg);
    //             ik_solver_foot_positions_latest_ = msg;
    //             // RCLCPP_INFO(this->get_logger(), "New front: %f, %f, %f, %f, %f, %f", 
    //             //     msg.left_foot_position.x, msg.left_foot_position.y, msg.left_foot_position.z, 
    //             //     msg.right_foot_position.x, msg.right_foot_position.y, msg.right_foot_position.z);
    //         }
    //         else
    //         {
    //             // Publish the IK solver foot positions.
    //             march_shared_msgs::msg::IksFootPositions msg;
    //             msg.header.stamp = this->now();
    //             msg.left_foot_position.x = ik_solver_foot_positions_buffer_.front().left_foot_position.x;
    //             msg.left_foot_position.y =itions msg;
    //     msg.header.stamp = this->now();
    //     msg.left_foot_position.x = ik_solver_foot_positions_latest_.left_foot_position.x;
    //     msg.left_foot_position.y = ik_solver_foot_positions_latest_.left_foot_position.y;
    //     msg.left_foot_position.z = ik_solver_foot_positions_latest_.left_foot_position.z;
    //     msg.right_foot_position.x = ik_solver_foot_positions_latest_.right_foot_position.x;
    //     msg.right_foot_position.y = ik_solver_foot_positions_latest_.right_foot_position.y;
    //     msg.right_foot_position.z = ik_solver_foot_positions_latest_.right_foot_position.z;
    //     msg.time_from_start.sec = 0;
    //             ik_solver_foot_positions_pub_->publish(msg);
    //             ik_solver_foot_positions_latest_ = msg;
    //             // RCLCPP_INFO(this->get_logger(), "Old front: %f, %f, %f, %f, %f, %f", 
    //             //     msg.left_foot_position.x, msg.left_foot_position.y, msg.left_foot_position.z, 
    //             //     msg.right_foot_position.x, msg.right_foot_position.y, msg.right_foot_position.z);
    //         }
    //     }
    // }
    // else
    // {
    //     // Publish the IK solver foot positions.
    //     march_shared_msgs::msg::IksFootPositions msg;
    //     msg.header.stamp = this->now();
    //     msg.left_foot_position.x = ik_solver_foot_positions_latest_.left_foot_position.x;
    //     msg.left_foot_position.y = ik_solver_foot_positions_latest_.left_foot_position.y;
    //     msg.left_foot_position.z = ik_solver_foot_positions_latest_.left_foot_position.z;
    //     msg.right_foot_position.x = ik_solver_foot_positions_latest_.right_foot_position.x;
    //     msg.right_foot_position.y = ik_solver_foot_positions_latest_.right_foot_position.y;
    //     msg.right_foot_position.z = ik_solver_foot_positions_latest_.right_foot_position.z;
    //     msg.time_from_start.sec = 0;
    //     msg.time_from_start.nanosec = (uint32_t) (dt_ * 1e9);
    //     ik_solver_foot_positions_pub_->publish(msg);
    // }

    // Publish the IK solver foot positions.
    march_shared_msgs::msg::IksFootPositions msg;
    msg.header.stamp = this->now();
    msg.left_foot_position.x = ik_solver_foot_positions_latest_.left_foot_position.x;
    msg.left_foot_position.y = ik_solver_foot_positions_latest_.left_foot_position.y;
    msg.left_foot_position.z = ik_solver_foot_positions_latest_.left_foot_position.z;
    msg.right_foot_position.x = ik_solver_foot_positions_latest_.right_foot_position.x;
    msg.right_foot_position.y = ik_solver_foot_positions_latest_.right_foot_position.y;
    msg.right_foot_position.z = ik_solver_foot_positions_latest_.right_foot_position.z;
    msg.time_from_start.sec = 0;
    msg.time_from_start.nanosec = (uint32_t) (dt_ * 1e9);
    msg.new_point = false;
    ik_solver_foot_positions_pub_->publish(msg);

    // Publish the IK solver status.
    publishIKSolverStatus();
}

void IKSolverBufferNode::publishIKSolverStatus()
{
    // Publish the IK solver status.
    std_msgs::msg::UInt32 msg;
    msg.data = ik_solver_foot_positions_buffer_.size();
    ik_solver_status_pub_->publish(msg);
}

void IKSolverBufferNode::publishIKSolverError(double error)
{
    // Publish the IK solver error.
    std_msgs::msg::Float64 msg;
    msg.data = error;
    ik_solver_error_pub_->publish(msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IKSolverBufferNode>());
    rclcpp::shutdown();
    return 0;
}