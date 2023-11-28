#include "ik_solver/ik_solver.hpp"

IKSolver::IKSolver()
{
    task_ = Task(1, "Test", 3, 4);

    // Define the proportional gain.
    task_.setGainP(1.0);

    // Calculate the Jacobian.
    task_.calculateJacobian();

    // Define the desired pose.
    Eigen::VectorXf desired_pose;
    desired_pose.resize(task_.getTaskM());
    desired_pose << 4.0, 5.0, 6.0;

    // Define the current pose.
    Eigen::VectorXf current_pose;
    current_pose.resize(task_.getTaskM());
    current_pose << 1.0, 2.0, 3.0;

    // Set the desired pose.
    task_.setDesiredPose(&desired_pose);

    // Set the current pose.
    task_.setCurrentPose(&current_pose);

    // Solve the task.
    joint_config_ = task_.solve();
}

Eigen::VectorXf IKSolver::getJointConfig()
{
    // Return the joint configuration.
    return joint_config_;
}

Eigen::MatrixXf IKSolver::getJacobian()
{
    // Return the Jacobian.
    return *task_.getJacobian();
}

Eigen::MatrixXf IKSolver::getJacobianInverse()
{
    // Return the inverse of Jacobian.
    return *task_.getJacobianInverse();
}