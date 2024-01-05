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
    ~IKSolver() = default; // Default destructor.

    Eigen::VectorXd solve(); // Solve the IK problem.
    Eigen::VectorXd integrateJointVelocities(); // Integrate the joint velocities
    std::vector<double> getTasksError(); // Get the error of the tasks.

    void setNJoints(int n_joints); // Set the number of joints.
    void setJointLimits(std::vector<std::array<double,2>> joint_limits); // Set the joint limits.
    void setTasks(std::vector<Task> tasks); // Set the tasks.

    void configureTasks(std::vector<Eigen::VectorXd> * desired_poses_ptr); // Configure the tasks.
    void setIntegralDtPtr(uint32_t* integral_dt_ptr); // Set the pointer to the integral time step.
    void setCurrentJointPositionsPtr(
        Eigen::VectorXd* current_joint_positions_ptr,
        std::vector<std::string> * joint_names_ptr); // Set the pointer to the current joint positions.
    void setDesiredJointPositionsPtr(
        Eigen::VectorXd* desired_joint_positions_ptr); // Set the pointer to the desired joint positions.
    void setDesiredJointVelocitiesPtr(
        Eigen::VectorXd* desired_joint_velocities_ptr); // Set the pointer to the desired joint velocities.
    // void setDesiredPoses(std::vector<Eigen::VectorXd> * desired_poses); // Set the desired poses of the tasks.

    // std::vector<double> getPose(const Eigen::VectorXd * joint_positions); // Get the pose of the end-effector.
    // const Eigen::MatrixXd * getJacobianPtr(int task_id); // Get the Jacobian of a task
    // const Eigen::MatrixXd * getJacobianInversePtr(int task_id); // Get the inverse of Jacobian of a task

private:
    Eigen::VectorXd setJointLimits(Eigen::VectorXd desired_joint_positions); // Set the joint limits

    int n_joints_; // Number of joints. TODO: Load this from a YAML file?
    std::vector<Task> tasks_; // A stack of tasks, the order of which is the priority of the tasks.
    std::vector<std::array<double,2>> joint_limits_; // Joint limits. TODO: Load this from a YAML file? Or from the robot description?
    uint32_t* integral_dt_ptr_;
    Eigen::VectorXd* current_joint_positions_ptr_; // Pointer to the current joint positions.
    Eigen::VectorXd* desired_joint_positions_ptr_; // Pointer to the desired joint positions.
    Eigen::VectorXd* desired_joint_velocities_ptr_; // Pointer to the desired joint velocities.
};

#endif // IK_SOLVER__IK_SOLVER_HPP_