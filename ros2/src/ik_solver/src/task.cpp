#include "ik_solver/task.hpp"

Task::Task()
{
    // Empty default constructor.
}

Task::Task(int task_id, std::string task_name, int task_m, int task_n)
{
    // Initialize the task ID, task name, dimension of the task, and dimension of the joint space.
    task_id_ = task_id;
    task_name_ = task_name;
    task_m_ = task_m;
    task_n_ = task_n;
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

void Task::setDesiredPose(Eigen::VectorXf * desired_pose)
{
    // Set the desired pose of the task
    desired_pose_ = desired_pose;
}

void Task::setCurrentPose(Eigen::VectorXf * current_pose)
{
    // Set the current pose of the task
    current_pose_ = current_pose;
}

Eigen::VectorXf Task::solve()
{
    // Solve the task
    Eigen::VectorXf joint_config;

    // Calculate the error
    Eigen::VectorXf error = calculateError();

    // Calculate the inverse of Jacobian.
    calculateJacobianInverse();

    // Calculate the joint configuration
    joint_config = jacobian_inverse_ * error;

    return joint_config;
}

Eigen::VectorXf Task::calculateError()
{
    // Define the error.
    Eigen::VectorXf error;

    // Calculate the error.
    error = *desired_pose_ - *current_pose_;

    // Calculate the proportional error.
    error = gain_p_ * error;

    return error;
}

// Eigen::VectorXf Task::calculateIntegralError()
// {
//     // Define the integral error.
//     Eigen::VectorXf integral_error;
//
//     // Calculate the integral error.
//     integral_error = integral_error_ + *desired_pose_ - *current_pose_;
//
//     // Update the integral error.
//     integral_error_ = integral_error;
//
//     return gain_i_ * integral_error;
// }

// Eigen::VectorXf Task::calculateDerivativeError()
// {
//     // Define the derivative error.
//     Eigen::VectorXf derivative_error;
//
//     // Calculate the derivative error.
//     derivative_error = *desired_pose_ - *current_pose_;
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

const Eigen::MatrixXf * Task::getJacobian()
{
    // Return the pointer of Jacobian
    return &jacobian_;
}

const Eigen::MatrixXf * Task::getJacobianInverse()
{
    // Return the pointer of  inverse of Jacobian
    return &jacobian_inverse_;
}

// void Task::setJacobian(Eigen::MatrixXf J)
// {
//     // Set the Jacobian
//     jacobian_ = jacobian;
// }

// void Task::setJacobianInverse(Eigen::MatrixXf jacobian_inverse_)
// {
//     // Set the inverse of Jacobian
//     jacobian_inverse_ = jacobian_inverse_;
// }

void Task::calculateJacobian()
{
    // Define the Jacobian.
    Eigen::MatrixXf jacobian;

    // Calculate the Jacobian.
    jacobian_ = Eigen::MatrixXf::Random(task_m_, task_n_);
}

void Task::calculateJacobianInverse()
{
    // Define the transpose of Jacobian.
    Eigen::MatrixXf jacobian_transpose;
    jacobian_transpose = jacobian_.transpose();

    // If the system is underdetermined.
    if (task_m_ < task_n_)
    {
        // Calculate the inverse using the pseudo-inverse.
        jacobian_inverse_ = jacobian_transpose * (jacobian_ * jacobian_transpose).inverse();
    }
    else // if the system is overdetermined.
    {
        // Calculate the inverse using the pseudo-inverse.
        jacobian_inverse_ = (jacobian_transpose * jacobian_).inverse() * jacobian_transpose;
    }

}