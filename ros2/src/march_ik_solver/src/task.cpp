#include "march_ik_solver/task.hpp"

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

int Task::getTaskID()
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

void Task::setDesiredPose(Eigen::VectorXd* desired_pose_ptr)
{
    // Set the desired pose of the task
    desired_pose_ptr_ = desired_pose_ptr;
}

Eigen::VectorXd Task::solve()
{
    // Send request to the task server in the state estimation node.
    sendRequest();

    // Calculate the error.
    Eigen::VectorXd error = calculateError();
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Error: %f, %f, %f, %f, %f, %f, %f, %f", error(0), error(1), error(2), error(3), error(4), error(5), error(6), error(7));

    // Calculate the inverse of Jacobian.
    calculateJacobianInverse();

    // Calculate the joint velocities.
    Eigen::VectorXd joint_velocities = jacobian_inverse_ * error;

    return joint_velocities;
}

Eigen::VectorXd Task::calculateError()
{
    // Define the error.
    Eigen::VectorXd error;

    // Calculate the error.
    error = *desired_pose_ptr_ - current_pose_;

    // Calculate the proportional error.
    error = gain_p_ * error; // + calculateIntegralError() + calculateDerivativeError();

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