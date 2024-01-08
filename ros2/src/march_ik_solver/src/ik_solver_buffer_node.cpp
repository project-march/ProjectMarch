#include "march_ik_solver/ik_solver_buffer_node.hpp"

IKSolverBufferNode::IKSolverBufferNode()
    : Node("ik_solver_buffer", rclcpp::NodeOptions())
{
    // Declare the node parameters.
    declare_parameter("dt", 0.01);
    declare_parameter("joints_size", 8);
    declare_parameter("convergence_threshold", 0.001);
    declare_parameter("early_stopping", false);
    declare_parameter("m_early_stoppingthreshold", 5);

    // Initialize the node parameters.
    m_n_joints = get_parameter("joints_size").as_int();
    m_dt = get_parameter("dt").as_double();
    m_convergence_threshold = get_parameter("convergence_threshold").as_double();
    m_early_stopping = get_parameter("early_stopping").as_bool();
    m_early_stopping_threshold = get_parameter("m_early_stoppingthreshold").as_int();
    RCLCPP_INFO(this->get_logger(), "dt: %f", m_dt);
    RCLCPP_INFO(this->get_logger(), "convergence_threshold: %f", m_convergence_threshold);

    // Create a timer that will publish the joint trajectory buffer.
    auto timer_callback = std::bind(&IKSolverBufferNode::publishIKSolverFootPositions, this);
    m_timer = this->create_wall_timer(std::chrono::milliseconds((int)(m_dt * 1000)), timer_callback);

    // Create a subscription to the joint states.
    m_joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&IKSolverBufferNode::jointStateCallback, this, std::placeholders::_1));

    // Create a subscription to the joint trajectory.
    m_joint_trajectory_sub = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "joint_trajectory_controller/joint_trajectory", 10, std::bind(&IKSolverBufferNode::jointTrajectoryCallback, this, std::placeholders::_1));

    // Create a subscription to the Exo state.
    m_exo_state_sub = this->create_subscription<march_shared_msgs::msg::ExoState>(
        "current_state", 10, std::bind(&IKSolverBufferNode::exoStateCallback, this, std::placeholders::_1));

    // Create a subscription to the IK solver foot positions.
    m_ik_solver_foot_positions_sub = this->create_subscription<march_shared_msgs::msg::IksFootPositions>(
        "ik_solver/buffer/input", 10, std::bind(&IKSolverBufferNode::ikSolverFootPositionsCallback, this, std::placeholders::_1));

    // Create a publisher for the IK solver foot positions.
    m_ik_solver_foot_positions_pub = this->create_publisher<march_shared_msgs::msg::IksFootPositions>(
        "ik_solver/buffer/output", 10);

    // Create a publisher for the IK solver status.
    m_ik_solver_status_pub = this->create_publisher<std_msgs::msg::UInt32>(
        "ik_solver/status", 1);
    m_ik_solver_error_pub = this->create_publisher<std_msgs::msg::Float64>(
        "ik_solver/error", 1);

    // Set the gait type to -1 as None, and set the gait reset to passthrough the first IK solver foot positions.
    m_gait_type = -1;
    m_gait_reset = true;
    m_ik_solver_foot_positions_received = false;
}

void IKSolverBufferNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Store the current joint positions.
    // m_current_joint_positions = Eigen::Map<Eigen::VectorXd>(msg->position.data(), m_n_joints);
    Eigen::VectorXd current_joint_positions = Eigen::VectorXd::Zero(m_n_joints);
    current_joint_positions(0) = msg->position[3];
    current_joint_positions(1) = msg->position[0];
    current_joint_positions(2) = msg->position[1];
    current_joint_positions(3) = msg->position[2];
    current_joint_positions(4) = msg->position[7];
    current_joint_positions(5) = msg->position[4];
    current_joint_positions(6) = msg->position[5];
    current_joint_positions(7) = msg->position[6];
    m_current_joint_positions = current_joint_positions;
}

void IKSolverBufferNode::jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
    // Store the joint trajectory in the buffer.
    m_desired_joint_positions = Eigen::Map<Eigen::VectorXd>(msg->points.back().positions.data(), m_n_joints);
}

void IKSolverBufferNode::exoStateCallback(const march_shared_msgs::msg::ExoState::SharedPtr msg)
{
    // Store the gait type.
    m_gait_type = msg->state;

    // Reset the gait reset flag.
    m_gait_reset = true;

    // Clear the buffer.
    m_ik_solver_foot_positions_buffer.clear();
}

void IKSolverBufferNode::ikSolverFootPositionsCallback(const march_shared_msgs::msg::IksFootPositions::SharedPtr msg)
{
    // Store the IK solver foot positions in the buffer.
    // m_ik_solver_foot_positions_buffer.push_back(*msg);
    m_ik_solver_foot_positions_latest = *msg;
    m_ik_solver_foot_positions_received = true;

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
    msg_out.time_from_start.nanosec = (uint32_t) (m_dt * 1e9);
    msg_out.new_point = true;
    m_ik_solver_foot_positions_pub->publish(msg_out);
}

void IKSolverBufferNode::publishIKSolverFootPositions()
{
    // Check if the gait type is set.
    if (m_gait_type == -1 || !m_ik_solver_foot_positions_received)
    {
        return;
    }

    // // Check if the buffer is empty.
    // if (m_ik_solver_foot_positions_buffer.size() > 0)
    // {
    //     // Check if the gait reset flag is set.
    //     if (m_gait_reset)
    //     {
    //         // Reset the gait reset flag.
    //         m_gait_reset = false;

    //         // // Set the current joint positions to the first IK solver foot positions.
    //         // m_current_joint_positions = Eigen::Map<Eigen::VectorXd>(m_ik_solver_foot_positions_buffer.front().positions.data(), m_n_joints);

    //         // Publish the IK solver foot positions.
    //         march_shared_msgs::msg::IksFootPositions msg;
    //         msg.header.stamp = this->now();
    //         msg.left_foot_position.x = m_ik_solver_foot_positions_buffer.front().left_foot_position.x;
    //         msg.left_foot_position.y = m_ik_solver_foot_positions_buffer.front().left_foot_position.y;
    //         msg.left_foot_position.z = m_ik_solver_foot_positions_buffer.front().left_foot_position.z;
    //         msg.right_foot_position.x = m_ik_solver_foot_positions_buffer.front().right_foot_position.x;
    //         msg.right_foot_position.y = m_ik_solver_foot_positions_buffer.front().right_foot_position.y;
    //         msg.right_foot_position.z = m_ik_solver_foot_positions_buffer.front().right_foot_position.z; 
    //         msg.time_from_start.sec = 0;
    //         msg.time_from_start.nanosec = 50;
    //         m_ik_solver_foot_positions_pub->publish(msg);

    //         // Clear the buffer.
    //         m_ik_solver_foot_positions_buffer.clear();
    //     }
    //     else
    //     {
    //         // Calculate the joint position error.
    //         double error = (m_desired_joint_positions - m_current_joint_positions).norm();

    //         // Publish the IK solver error.
    //         publishIKSolverError(error);

    //         // Check if the desired joint positions are reached.
    //         // RCLCPP_INFO(this->get_logger(), "Error: %f", error);
    //         // RCLCPP_INFO(this->get_logger(), "Convergence threshold: %f", m_convergence_threshold);
    //         if (m_early_stopping || (m_ik_solver_foot_positions_buffer.size() > m_early_stopping_threshold))
    //         {
    //             march_shared_msgs::msg::IksFootPositions msg;
    //             msg.header.stamp = this->now();
    //             msg.left_foot_position.x = m_ik_solver_foot_positions_buffer.back().left_foot_position.x;
    //             msg.left_foot_position.y = m_ik_solver_foot_positions_buffer.back().left_foot_position.y;
    //             msg.left_foot_position.z = m_ik_solver_foot_positions_buffer.back().left_foot_position.z;
    //             msg.right_foot_position.x = m_ik_solver_foot_positions_buffer.back().right_foot_position.x;
    //             msg.right_foot_position.y = m_ik_solver_foot_positions_buffer.back().right_foot_position.y;
    //             msg.right_foot_position.z = m_ik_solver_foot_positions_buffer.back().right_foot_position.z; 
    //             msg.time_from_start.sec = 0;
    //             msg.time_from_start.nanosec = (uint32_t) (m_dt * 1e9);
    //             m_ik_solver_foot_positions_pub->publish(msg);

    //             // Clear the buffer.
    //             m_ik_solver_foot_positions_buffer.clear();
    //             ik_solver_foot_potion.x = m_ik_solver_foot_positions_buffer.front().left_foot_position.x;
    //             msg.left_foot_position.y = m_ik_solver_foot_positions_buffer.front().left_foot_position.y;
    //             msg.left_foot_position.z = m_ik_solver_foot_positions_buffer.front().left_foot_position.z;
    //             msg.right_foot_position.x = m_ik_solver_foot_positions_buffer.front().right_foot_position.x;
    //             msg.right_foot_position.y = m_ik_solver_foot_positions_buffer.front().right_foot_position.y;
    //             msg.right_foot_position.z = m_ik_solver_foot_positions_buffer.front().right_foot_position.z; 
    //             msg.time_from_start.sec = 0;
    //             msg.time_from_start.nanosec = (uint32_t) (m_dt * 1e9);
    //             m_ik_solver_foot_positions_pub->publish(msg);
    //             m_ik_solver_foot_positions_latest = msg;
    //             // RCLCPP_INFO(this->get_logger(), "New front: %f, %f, %f, %f, %f, %f", 
    //             //     msg.left_foot_position.x, msg.left_foot_position.y, msg.left_foot_position.z, 
    //             //     msg.right_foot_position.x, msg.right_foot_position.y, msg.right_foot_position.z);
    //         }
    //         else
    //         {
    //             // Publish the IK solver foot positions.
    //             march_shared_msgs::msg::IksFootPositions msg;
    //             msg.header.stamp = this->now();
    //             msg.left_foot_position.x = m_ik_solver_foot_positions_buffer.front().left_foot_position.x;
    //             msg.left_foot_position.y =itions msg;
    //     msg.header.stamp = this->now();
    //     msg.left_foot_position.x = m_ik_solver_foot_positions_latest.left_foot_position.x;
    //     msg.left_foot_position.y = m_ik_solver_foot_positions_latest.left_foot_position.y;
    //     msg.left_foot_position.z = m_ik_solver_foot_positions_latest.left_foot_position.z;
    //     msg.right_foot_position.x = m_ik_solver_foot_positions_latest.right_foot_position.x;
    //     msg.right_foot_positiontion.x = m_ik_solver_foot_positions_buffer.front().left_foot_position.x;
    //             msg.left_foot_position.y = m_ik_solver_foot_positions_buffer.front().left_foot_position.y;
    //             msg.left_foot_position.z = m_ik_solver_foot_positions_buffer.front().left_foot_position.z;
    //             msg.right_foot_position.x = m_ik_solver_foot_positions_buffer.front().right_foot_position.x;
    //             msg.right_foot_position.y = m_ik_solver_foot_positions_buffer.front().right_foot_position.y;
    //             msg.right_foot_position.z = m_ik_solver_foot_positions_buffer.front().right_foot_position.z; 
    //             msg.time_from_start.sec = 0;
    //             msg.time_from_start.nanosec = (uint32_t) (m_dt * 1e9);
    //             m_ik_solver_foot_positions_pub->publish(msg);
    //             m_ik_solver_foot_positions_latest = msg;
    //             // RCLCPP_INFO(this->get_logger(), "New front: %f, %f, %f, %f, %f, %f", 
    //             //     msg.left_foot_position.x, msg.left_foot_position.y, msg.left_foot_position.z, 
    //             //     msg.right_foot_position.x, msg.right_foot_position.y, msg.right_foot_position.z);
    //         }
    //         else
    //         {
    //             // Publish the IK solver foot positions.
    //             march_shared_msgs::msg::IksFootPositions msg;
    //             msg.header.stamp = this->now();
    //             msg.left_foot_position.x = m_ik_solver_foot_positions_buffer.front().left_foot_position.x;
    //             msg.left_foot_position.y =itions msg;
    //     msg.header.stamp = this->now();
    //     msg.left_foot_position.x = m_ik_solver_foot_positions_latest.left_foot_position.x;
    //     msg.left_foot_position.y = m_ik_solver_foot_positions_latest.left_foot_position.y;
    //     msg.left_foot_position.z = m_ik_solver_foot_positions_latest.left_foot_position.z;
    //     msg.right_foot_position.x = m_ik_solver_foot_positions_latest.right_foot_position.x;
    //     msg.right_foot_position.y = m_ik_solver_foot_positions_latest.right_foot_position.y;
    //     msg.right_foot_position.z = m_ik_solver_foot_positions_latest.right_foot_position.z;
    //     msg.time_from_start.sec = 0;
    //             m_ik_solver_foot_positions_pub->publish(msg);
    //             m_ik_solver_foot_positions_latest = msg;
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
    //     msg.left_foot_position.x = m_ik_solver_foot_positions_latest.left_foot_position.x;
    //     msg.left_foot_position.y = m_ik_solver_foot_positions_latest.left_foot_position.y;
    //     msg.left_foot_position.z = m_ik_solver_foot_positions_latest.left_foot_position.z;
    //     msg.right_foot_position.x = m_ik_solver_foot_positions_latest.right_foot_position.x;
    //     msg.right_foot_position.y = m_ik_solver_foot_positions_latest.right_foot_position.y;
    //     msg.right_foot_position.z = m_ik_solver_foot_positions_latest.right_foot_position.z;
    //     msg.time_from_start.sec = 0;
    //     msg.time_from_start.nanosec = (uint32_t) (m_dt * 1e9);
    //     msg.right_foot_position.z = m_ik_solver_foot_positions_latest.right_foot_position.z;
    //     msg.time_from_start.sec = 0;
    //             m_ik_solver_foot_positions_pub->publish(msg);
    //             m_ik_solver_foot_positions_latest = msg;
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
    //     msg.left_foot_position.x = m_ik_solver_foot_positions_latest.left_foot_position.x;
    //     msg.left_foot_position.y = m_ik_solver_foot_positions_latest.left_foot_position.y;
    //     msg.left_foot_position.z = m_ik_solver_foot_positions_latest.left_foot_position.z;
    //     msg.right_foot_position.x = m_ik_solver_foot_positions_latest.right_foot_position.x;
    //     msg.right_foot_position.y = m_ik_solver_foot_positions_latest.right_foot_position.y;
    //     msg.right_foot_position.z = m_ik_solver_foot_positions_latest.right_foot_position.z;
    //     msg.time_from_start.sec = 0;
    //     msg.time_from_start.nanosec = (uint32_t) (m_dt * 1e9);
    //     m_ik_solver_foot_positions_pub->publish(msg);sitions_latest_ = msg;
    //             // RCLCPP_INFO(this->get_logger(), "Early stopping");
    //             return;
    //         }
    //         if (error < m_convergence_threshold)
    //         {
        //             // Remove the first IK solver foot positions from the buffer.
        //             m_ik_solver_foot_positions_buffer.erase(m_ik_solver_foot_positions_buffer.begin());

        //             march_shared_msgs::msg::IksFootPositions msg;
        //             msg.header.stamp = this->now();
        //             msg.left_foot_position.x = m_ik_solver_foot_positions_buffer.front().left_foot_position.x;
        //             msg.left_foot_position.y = m_ik_solver_foot_positions_buffer.front().left_foot_position.y;
        //             msg.left_foot_position.z = m_ik_solver_foot_positions_buffer.front().left_foot_position.z;
        //             msg.right_foot_position.x = m_ik_solver_foot_positions_buffer.front().right_foot_position.x;
        //             msg.right_foot_position.y = m_ik_solver_foot_positions_buffer.front().right_foot_position.y;
        //             msg.right_foot_position.z = m_ik_solver_foot_positions_buffer.front().right_foot_position.z; 
        //             msg.time_from_start.sec = 0;
        //             msg.time_from_start.nanosec = (uint32_t) (m_dt * 1e9);
        //             m_ik_solver_foot_positions_pub->publish(msg);
        //             m_ik_solver_foot_positions_latest = msg;
        //             // RCLCPP_INFO(this->get_logger(), "New front: %f, %f, %f, %f, %f, %f", 
        //             //     msg.left_foot_position.x, msg.left_foot_position.y, msg.left_foot_position.z, 
        //             //     msg.right_foot_position.x, msg.right_foot_position.y, msg.right_foot_position.z);
        //         }
        //         else
        //         {
        //             // Publish the IK solver foot positions.
        //             march_shared_msgs::msg::IksFootPositions msg;
        //             msg.header.stamp = this->now();
        //             msg.left_foot_position.x = m_ik_solver_foot_positions_buffer.front().left_foot_position.x;
        //             msg.left_foot_position.y = m_ik_solver_foot_positions_buffer.front().left_foot_position.y;
        //             msg.left_foot_position.z = m_ik_solver_foot_positions_buffer.front().left_foot_position.z;
        //             msg.right_foot_position.x = m_ik_solver_foot_positions_buffer.front().right_foot_position.x;
        //             msg.right_foot_position.y = m_ik_solver_foot_positions_buffer.front().right_foot_position.y;
        //             msg.right_foot_position.z = m_ik_solver_foot_positions_buffer.front().right_foot_position.z;
        //             msg.time_from_start.sec = 0;
        //             m_ik_solver_foot_positions_pub->publish(msg);
        //             m_ik_solver_foot_positions_latest = msg;
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
        //     msg.left_foot_position.x = m_ik_solver_foot_positions_latest.left_foot_position.x;
        //     msg.left_foot_position.y = m_ik_solver_foot_positions_latest.left_foot_position.y;
        //     msg.left_foot_position.z = m_ik_solver_foot_positions_latest.left_foot_position.z;
        //     msg.right_foot_position.x = m_ik_solver_foot_positions_latest.right_foot_position.x;
        //     msg.right_foot_position.y = m_ik_solver_foot_positions_latest.right_foot_position.y;
        //     msg.right_foot_position.z = m_ik_solver_foot_positions_latest.right_foot_position.z;
        //     msg.time_from_start.sec = 0;
        //     msg.time_from_start.nanosec = (uint32_t) (m_dt * 1e9);
        //     m_ik_solver_foot_positions_pub->publish(msg);
        // }

        // Publish the IK solver foot positions.
        march_shared_msgs::msg::IksFootPositions msg;
        msg.header.stamp = this->now();
        msg.left_foot_position.x = m_ik_solver_foot_positions_latest.left_foot_position.x;
        msg.left_foot_position.y = m_ik_solver_foot_positions_latest.left_foot_position.y;
        msg.left_foot_position.z = m_ik_solver_foot_positions_latest.left_foot_position.z;
        msg.right_foot_position.x = m_ik_solver_foot_positions_latest.right_foot_position.x;
        msg.right_foot_position.y = m_ik_solver_foot_positions_latest.right_foot_position.y;
        msg.right_foot_position.z = m_ik_solver_foot_positions_latest.right_foot_position.z;
        msg.time_from_start.sec = 0;
        msg.time_from_start.nanosec = (uint32_t) (m_dt * 1e9);
        msg.new_point = false;
        m_ik_solver_foot_positions_pub->publish(msg);

        // Publish the IK solver status.
        publishIKSolverStatus();
    }

    void IKSolverBufferNode::publishIKSolverStatus()
    {
        // Publish the IK solver status.
        std_msgs::msg::UInt32 msg;
        msg.data = m_ik_solver_foot_positions_buffer.size();
        m_ik_solver_status_pub->publish(msg);
    }

    void IKSolverBufferNode::publishIKSolverError(double error)
    {
        // Publish the IK solver error.
        std_msgs::msg::Float64 msg;
        msg.data = error;
        m_ik_solver_error_pub->publish(msg);
    }

    int main(int argc, char * argv[])
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<IKSolverBufferNode>());
        rclcpp::shutdown();
        return 0;    }

