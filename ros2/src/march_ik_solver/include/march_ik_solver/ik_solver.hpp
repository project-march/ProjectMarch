#ifndef IK_SOLVER__IK_SOLVER_HPP_
#define IK_SOLVER__IK_SOLVER_HPP_

#pragma once
#include <vector>

#include "march_ik_solver/task.hpp"
#include "rclcpp/rclcpp.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

class IKSolver {
public:
    IKSolver(); // Default constructor.
    ~IKSolver(); // Default destructor.

    Eigen::VectorXd solve(std::vector<Eigen::VectorXd> desired_poses); // Solve the IK problem.
    Eigen::VectorXd integrateJointVelocities(); // Integrate the joint velocities

    void setDt(double dt);
    void setNJoints(int n_joints); // Set the number of joints.
    void setJointLimits(std::vector<std::array<double,2>> joint_limits); // Set the joint limits.
    void setTasks(std::vector<Task> tasks); // Set the tasks.

    uint8_t getNumberOfTasks(); // Get the number of tasks.
    std::vector<uint8_t> getTasksM(); // Get the dimension of each task.

    void setCurrentJointPositionsPtr(
        Eigen::VectorXd* current_joint_positions_ptr); // Set the pointer to the current joint positions.
    void setDesiredJointPositionsPtr(
        Eigen::VectorXd* desired_joint_positions_ptr); // Set the pointer to the desired joint positions.
    void setDesiredJointVelocitiesPtr(
        Eigen::VectorXd* desired_joint_velocities_ptr); // Set the pointer to the desired joint velocities.
    // void setDesiredPoses(std::vector<Eigen::VectorXd> * desired_poses); // Set the desired poses of the tasks.

    // const Eigen::MatrixXd * getJacobianPtr(int task_id); // Get the Jacobian of a task
    // const Eigen::MatrixXd * getJacobianInversePtr(int task_id); // Get the inverse of Jacobian of a task

private:
    void configureTasks(); // Configure the tasks
    Eigen::VectorXd setJointLimits(Eigen::VectorXd desired_joint_positions); // Set the joint limits

    int n_joints_; // Number of joints. TODO: Load this from a YAML file?
    double dt_; // Time step. TODO: Load this from a YAML file?
    std::vector<Task> tasks_; // A stack of tasks, the order of which is the priority of the tasks.
    std::vector<std::array<double,2>> joint_limits_; // Joint limits. TODO: Load this from a YAML file? Or from the robot description?
    Eigen::VectorXd* current_joint_positions_ptr_; // Pointer to the current joint positions.
    Eigen::VectorXd* desired_joint_positions_ptr_; // Pointer to the desired joint positions.
    Eigen::VectorXd* desired_joint_velocities_ptr_; // Pointer to the desired joint velocities.
};

#endif // IK_SOLVER__IK_SOLVER_HPP_