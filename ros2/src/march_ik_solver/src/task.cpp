#include "march_ik_solver/task.hpp"
#include "math.h"

Task::Task()
{
    // Empty default constructor.
}

Task::Task(uint8_t task_id, std::string task_name, uint8_t task_m, uint8_t task_n)
{
    // Initialize the task ID, task name, dimension of the task, and dimension of the joint space.
    task_id_ = task_id;
    task_name_ = task_name;
    task_m_ = task_m;
    task_n_ = task_n;

    // Initialize the task client to communicate with the task server in the state estimation node.
    node_ = std::make_shared<rclcpp::Node>("task_" + task_name_ + "_client");
    client_ = node_->create_client<march_shared_msgs::srv::GetTaskReport>("get_task_report");
}

std::string Task::getTaskName()
{
    // Return the name of the task
    return task_name_;
}

unsigned int Task::getTaskID()
{
    // Return the ID of the task
    return task_id_;
}

int Task::getTaskM()
{
    // Return the dimension of the task
    return task_m_;
}

int Task::getTaskN()
{
    // Return the dimension of the joint space
    return task_n_;
}

Eigen::VectorXd Task::getPose(const Eigen::VectorXd * joint_positions)
{
    // Return the current pose of the task
    Eigen::VectorXd pose = Eigen::VectorXd::Zero(task_m_);

    // Calculate the current pose.
    Eigen::VectorXd q = *joint_positions;
    double q_LHAA = q(0);
    double q_LHFE = q(1);
    double q_LKFE = q(2);
    double q_LADPF = q(3);
    double q_RHAA = q(4);
    double q_RHFE = q(5);
    double q_RKFE = -q(6);
    double q_RADPF = q(7);

    // x(0) = 0.41*sin(q_LHFE - q_LKFE)*cos(q_LHAA) + 0.0562224*sin(q_LADPF + q_LHFE - q_LKFE)*cos(q_LHAA) + 0.4*sin(q_LHFE)*cos(q_LHAA) + 0.0562224*cos(q_LADPF + q_LHFE - q_LKFE) + 0.014;
    // x(1) = 0.8662224*sin(q_LHAA) + 0.16;
    // x(2) = 0.0562224*sin(q_LADPF + q_LHFE - q_LKFE) - 0.41*cos(q_LHFE - q_LKFE)*cos(q_LHAA) - 0.0562224*cos(q_LADPF + q_LHFE - q_LKFE)*cos(q_LHAA) - 0.4*cos(q_LHAA)*cos(q_LHFE) - 0.144;
    // x(3) = 0.41*sin(q_RHFE - q_RKFE)*cos(q_RHAA) + 0.0562224*sin(q_RADPF + q_RHFE - q_RKFE)*cos(q_RHAA) + 0.4*sin(q_RHFE)*cos(q_RHAA) + 0.0562224*cos(q_RADPF + q_RHFE - q_RKFE) + 0.014;
    // x(4) = 0.8662224*sin(q_RHAA) - 0.16;
    // x(5) = 0.0562224*sin(q_RADPF + q_RHFE - q_RKFE) - 0.41*cos(q_RHFE - q_RKFE)*cos(q_RHAA) - 0.0562224*cos(q_RADPF + q_RHFE - q_RKFE)*cos(q_RHAA) - 0.4*cos(q_RHAA)*cos(q_RHFE) - 0.144;
    pose(0) = 0.41*sin(q_LHFE - q_LKFE)*cos(q_LHAA) + 0.4*sin(q_LHFE)*cos(q_LHAA) + 0.014;
    pose(1) = 0.81*sin(q_LHAA) + 0.16;
    pose(2) = -0.41*cos(q_LHFE - q_LKFE)*cos(q_LHAA) - 0.4*cos(q_LHAA)*cos(q_LHFE) - 0.144;
    pose(3) = 0.41*sin(q_RHFE - q_RKFE)*cos(q_RHAA) + 0.4*sin(q_RHFE)*cos(q_RHAA) + 0.014;
    pose(4) = 0.81*sin(q_RHAA) - 0.16;
    pose(5) = -0.41*cos(q_RHFE - q_RKFE)*cos(q_RHAA) - 0.4*cos(q_RHAA)*cos(q_RHFE) - 0.144;

    return pose;
}

void Task::setCurrentJointPositionsPtr(Eigen::VectorXd * current_joint_positions_ptr)
{
    // Set the current joint positions
    current_joint_positions_ptr_ = current_joint_positions_ptr;
}

void Task::setDesiredPosesPtr(std::vector<Eigen::VectorXd> * desired_poses_ptr)
{
    // Set the desired pose of the task
    desired_poses_ptr_ = desired_poses_ptr;
}

Eigen::VectorXd Task::solve()
{
    // Send request to the task server in the state estimation node.
    // sendRequest();
    calculateCurrentPose();

    // Calculate the error.
    Eigen::VectorXd error = calculateError();
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Error: %f, %f, %f, %f, %f, %f, %f, %f", error(0), error(1), error(2), error(3), error(4), error(5), error(6), error(7));

    // Calculate the inverse of Jacobian.
    calculateJacobian();
    calculateJacobianInverse();

    // Print the sizes
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Jacobian size: %d, %d", jacobian_.rows(), jacobian_.cols());
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Jacobian inverse size: %d, %d", jacobian_inverse_.rows(), jacobian_inverse_.cols());
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Error size: %d", error.size());

    // Calculate the joint velocities.
    // Eigen::VectorXd joint_velocities = jacobian_inverse_ * error;
    Eigen::VectorXd joint_velocities = Eigen::VectorXd::Zero(task_n_);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint velocities size: %d", joint_velocities.size());
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint velocities: %f, %f, %f, %f, %f, %f, %f, %f", joint_velocities(0), joint_velocities(1), joint_velocities(2), joint_velocities(3), joint_velocities(4), joint_velocities(5), joint_velocities(6), joint_velocities(7));
    for (int i = 0; i < task_n_; i++)
    {
        for (int j = 0; j < task_m_; j++)
        {
            joint_velocities(i) += jacobian_inverse_(i,j) * error(j);
        }
    }
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint velocities: %f, %f, %f, %f, %f, %f, %f, %f", joint_velocities(0), joint_velocities(1), joint_velocities(2), joint_velocities(3), joint_velocities(4), joint_velocities(5), joint_velocities(6), joint_velocities(7));

    return joint_velocities;
}

Eigen::VectorXd Task::calculateError()
{
    // Define the error.
    Eigen::VectorXd error = Eigen::VectorXd::Zero(task_m_);

    // Calculate the error.
    error = (*desired_poses_ptr_)[task_id_] - current_pose_;
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Desired pose: %f, %f, %f, %f, %f, %f", (*desired_poses_ptr_)[task_id_](0), (*desired_poses_ptr_)[task_id_](1), (*desired_poses_ptr_)[task_id_](2), (*desired_poses_ptr_)[task_id_](3), (*desired_poses_ptr_)[task_id_](4), (*desired_poses_ptr_)[task_id_](5));
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current pose: %f, %f, %f, %f, %f, %f", current_pose_(0), current_pose_(1), current_pose_(2), current_pose_(3), current_pose_(4), current_pose_(5));
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Error: %f, %f, %f, %f, %f, %f", error(0), error(1), error(2), error(3), error(4), error(5));

    // Calculate the proportional error.
    error = gain_p_ * error; // + calculateIntegralError() + calculateDerivativeError();

    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Proportional error: %f, %f, %f, %f, %f, %f", error(0), error(1), error(2), error(3), error(4), error(5));

    return error;
}

// Eigen::VectorXd Task::calculateIntegralError()
// {
//     // Define the integral error.
//     Eigen::VectorXd integral_error;
//
//     // Calculate the integral error.
//     integral_error = integral_error_ + *desired_pose_ - current_pose_;
//
//     // Update the integral error.
//     integral_error_ = integral_error;
//
//     return gain_i_ * integral_error;
// }

// Eigen::VectorXd Task::calculateDerivativeError()
// {
//     // Define the derivative error.
//     Eigen::VectorXd derivative_error;
//
//     // Calculate the derivative error.
//     derivative_error = *desired_pose_ - current_pose_;
//
//     // Update the previous error.
//     previous_error_ = derivative_error;
//
//     return gain_d_ * derivative_error;
// }

void Task::setGainP(float gain_p)
{
    // Set the proportional gain
    gain_p_ = gain_p;
}

// void Task::setGainI(float gain_i_)
// {
//     // Set the integral gain
//     gain_i_ = gain_i_;
// }

// void Task::setGainD(float gain_d_)
// {
//     // Set the derivative gain
//     gain_d_ = gain_d_;
// }

void Task::setDampingCoefficient(float damping_coefficient)
{
    // Set the damping coefficient
    damping_coefficient_ = damping_coefficient;
}

const Eigen::MatrixXd* Task::getJacobianPtr()
{
    // Return the pointer of Jacobian
    sendRequest();
    return &jacobian_;
}

const Eigen::MatrixXd* Task::getJacobianInversePtr()
{
    // Return the pointer of inverse of Jacobian
    calculateJacobianInverse();
    return &jacobian_inverse_;
}

void Task::calculateCurrentPose()
{
    // Define the current pose.
    Eigen::VectorXd current_pose = Eigen::VectorXd::Zero(task_m_);

    // Calculate the current pose.
    Eigen::VectorXd q = *current_joint_positions_ptr_;
    double q_LHAA = q(0);
    double q_LHFE = q(1);
    double q_LKFE = q(2);
    double q_LADPF = q(3);
    double q_RHAA = q(4);
    double q_RHFE = q(5);
    double q_RKFE = -q(6);
    double q_RADPF = q(7);

    // x(0) = 0.41*sin(q_LHFE - q_LKFE)*cos(q_LHAA) + 0.0562224*sin(q_LADPF + q_LHFE - q_LKFE)*cos(q_LHAA) + 0.4*sin(q_LHFE)*cos(q_LHAA) + 0.0562224*cos(q_LADPF + q_LHFE - q_LKFE) + 0.014;
    // x(1) = 0.8662224*sin(q_LHAA) + 0.16;
    // x(2) = 0.0562224*sin(q_LADPF + q_LHFE - q_LKFE) - 0.41*cos(q_LHFE - q_LKFE)*cos(q_LHAA) - 0.0562224*cos(q_LADPF + q_LHFE - q_LKFE)*cos(q_LHAA) - 0.4*cos(q_LHAA)*cos(q_LHFE) - 0.144;
    // x(3) = 0.41*sin(q_RHFE - q_RKFE)*cos(q_RHAA) + 0.0562224*sin(q_RADPF + q_RHFE - q_RKFE)*cos(q_RHAA) + 0.4*sin(q_RHFE)*cos(q_RHAA) + 0.0562224*cos(q_RADPF + q_RHFE - q_RKFE) + 0.014;
    // x(4) = 0.8662224*sin(q_RHAA) - 0.16;
    // x(5) = 0.0562224*sin(q_RADPF + q_RHFE - q_RKFE) - 0.41*cos(q_RHFE - q_RKFE)*cos(q_RHAA) - 0.0562224*cos(q_RADPF + q_RHFE - q_RKFE)*cos(q_RHAA) - 0.4*cos(q_RHAA)*cos(q_RHFE) - 0.144;
    current_pose(0) = 0.41*sin(q_LHFE - q_LKFE)*cos(q_LHAA) + 0.4*sin(q_LHFE)*cos(q_LHAA) + 0.014;
    current_pose(1) = 0.81*sin(q_LHAA) + 0.16;
    current_pose(2) = -0.41*cos(q_LHFE - q_LKFE)*cos(q_LHAA) - 0.4*cos(q_LHAA)*cos(q_LHFE) - 0.144;
    current_pose(3) = 0.41*sin(q_RHFE - q_RKFE)*cos(q_RHAA) + 0.4*sin(q_RHFE)*cos(q_RHAA) + 0.014;
    current_pose(4) = 0.81*sin(q_RHAA) - 0.16;
    current_pose(5) = -0.41*cos(q_RHFE - q_RKFE)*cos(q_RHAA) - 0.4*cos(q_RHAA)*cos(q_RHFE) - 0.144;

    // Update the current pose.
    current_pose_ = current_pose;
}

void Task::calculateJacobian()
{
    // Define the Jacobian.
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(task_m_, task_n_);

    // Calculate the Jacobian.
    Eigen::VectorXd q = *current_joint_positions_ptr_;
    double q_LHAA = q(0);
    double q_LHFE = q(1);
    double q_LKFE = q(2);
    double q_LADPF = q(3);
    double q_RHAA = q(4);
    double q_RHFE = q(5);
    double q_RKFE = -q(6);
    double q_RADPF = q(7);

    // Left foot Jacobian.
    jacobian(0,0) = -0.41*sin(q_LHFE - q_LKFE)*sin(q_LHAA) - 0.4*sin(q_LHAA)*sin(q_LHFE);
    jacobian(0,1) = 0.41*cos(q_LHFE - q_LKFE)*cos(q_LHAA) + 0.4*cos(q_LHAA)*cos(q_LHFE);
    jacobian(0,2) = -0.41*cos(q_LHFE - q_LKFE)*cos(q_LHAA);

    jacobian(1,0) = 0.81*cos(q_LHAA);

    jacobian(2,0) = 0.41*sin(q_LHAA)*cos(q_LHFE - q_LKFE) + 0.4*sin(q_LHAA)*cos(q_LHFE);
    jacobian(2,1) = 0.41*sin(q_LHFE - q_LKFE)*cos(q_LHAA) + 0.4*sin(q_LHFE)*cos(q_LHAA);
    jacobian(2,2) = -0.41*sin(q_LHFE - q_LKFE)*cos(q_LHAA);

    // Right foot Jacobian.
    jacobian(3,4) = -0.41*sin(q_RHFE - q_RKFE)*sin(q_RHAA) - 0.4*sin(q_RHAA)*sin(q_RHFE);
    jacobian(3,5) = 0.41*cos(q_RHFE - q_RKFE)*cos(q_RHAA) + 0.4*cos(q_RHAA)*cos(q_RHFE);
    jacobian(3,6) = -0.41*cos(q_RHFE - q_RKFE)*cos(q_RHAA);

    jacobian(4,4) = 0.81*cos(q_RHAA);

    jacobian(5,4) = .41*sin(q_RHAA)*cos(q_RHFE - q_RKFE) + 0.4*sin(q_RHAA)*cos(q_RHFE);
    jacobian(5,6) = .41*sin(q_RHFE - q_RKFE)*cos(q_RHAA) + 0.4*sin(q_RHFE)*cos(q_RHAA);
    jacobian(5,7) = -0.41*sin(q_RHFE - q_RKFE)*cos(q_RHAA);

    // Round the Jacobian to zero if it is very small.
    for (int i = 0; i < task_m_; i++)
    {
        for (int j = 0; j < task_n_; j++)
        {
            if (jacobian(i,j) < 1e-6 && jacobian(i,j) > -1e-6)
            {
                jacobian(i,j) = 0.0;
            }
        }
    }

    // Print the Jacobian.
    // for (int i = 0; i < task_m_; i++)
    // {
    //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Jacobian row %d: %f, %f, %f, %f, %f, %f, %f, %f", i, jacobian(i,0), jacobian(i,1), jacobian(i,2), jacobian(i,3), jacobian(i,4), jacobian(i,5), jacobian(i,6), jacobian(i,7));
    // }

    // Update the Jacobian.
    jacobian_ = jacobian;
}

void Task::calculateJacobianInverse()
{
    // Define the transpose of Jacobian.
    Eigen::MatrixXd jacobian_transpose = jacobian_.transpose();

    // If the system is underdetermined.
    if (task_m_ < task_n_) {
        // Damping matrix.
        Eigen::MatrixXd damping_matrix = damping_coefficient_ * Eigen::MatrixXd::Identity(task_m_, task_m_);

        // Calculate the inverse using the pseudo-inverse.
        jacobian_inverse_ = jacobian_transpose * (jacobian_ * jacobian_transpose + damping_matrix).inverse();
    } else // if the system is overdetermined.
    {
        // Damping matrix.
        Eigen::MatrixXd damping_matrix = damping_coefficient_ * Eigen::MatrixXd::Identity(task_n_, task_n_);

        // Calculate the inverse using the pseudo-inverse.
        jacobian_inverse_ = (jacobian_transpose * jacobian_ + damping_matrix).inverse() * jacobian_transpose;
    }

    // jacobian_inverse_ = (jacobian_transpose * (jacobian_ * jacobian_transpose).inverse()) * (task_m_ < task_n_) +
    //     ((jacobian_transpose * jacobian_).inverse() * jacobian_transpose) * (task_m_ >= task_n_);

    // Round the Jacobian inverse to zero if it is very small.
    for (int i = 0; i < task_n_; i++)
    {
        for (int j = 0; j < task_m_; j++)
        {
            if (jacobian_inverse_(i,j) < 1e-6 && jacobian_inverse_(i,j) > -1e-6)
            {
                jacobian_inverse_(i,j) = 0.0;
            }
        }
    }

    // Print the Jacobian inverse.
    // for (int i = 0; i < task_n_; i++)
    // {
    //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Jacobian inverse row %d: %f, %f, %f, %f, %f, %f", i, jacobian_inverse_(i,0), jacobian_inverse_(i,1), jacobian_inverse_(i,2), jacobian_inverse_(i,3), jacobian_inverse_(i,4), jacobian_inverse_(i,5));
    // }
}

void Task::sendRequest()
{
    auto request = std::make_shared<march_shared_msgs::srv::GetTaskReport::Request>();

    // TODO: To remove the following lines.
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client_->async_send_request(request);

    // TODO: Add error handling.
    if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), result)
        == rclcpp::FutureReturnCode::SUCCESS) {
        // Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(task_m_, task_n_);
        // Eigen::Map<const Eigen::VectorXd> jacobian_vector(result.get()->jacobian.data(),
        // result.get()->jacobian.size()); std::vector<float> jacobian_vector(result.get()->jacobian.begin(),
        // result.get()->jacobian.end());
        Eigen::MatrixXd jacobian = Eigen::Map<Eigen::MatrixXd>(result.get()->jacobian.data(), task_m_, task_n_);
        jacobian_ = jacobian;

        // std::vector<float> current_pose_vector(result.get()->current_pose.begin(), result.get()->current_pose.end());
        Eigen::VectorXd current_pose = Eigen::Map<Eigen::VectorXd>(result.get()->current_pose.data(), task_m_);
        current_pose_ = current_pose;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service get_task_report");
    }
}