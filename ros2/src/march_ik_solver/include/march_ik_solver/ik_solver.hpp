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
    IKSolver() = default;
    ~IKSolver() = default;

    Eigen::VectorXd solve();
    Eigen::VectorXd integrateJointVelocities();
    std::vector<double> getTasksError();

    void setNJoints(int n_joints);
    void setJointLimits(std::vector<double> lower_joint_limits, std::vector<double> upper_joint_limits);
    void setTasks(std::vector<std::shared_ptr<Task>> tasks);

    void configureTasks(std::vector<Eigen::VectorXd> * desired_poses_ptr);
    void setIntegralDtPtr(uint32_t* integral_dt_ptr);
    void setCurrentJointPositionsPtr(
        Eigen::VectorXd* current_joint_positions_ptr,
        std::vector<std::string> * joint_names_ptr);
    void setDesiredJointPositionsPtr(
        Eigen::VectorXd* desired_joint_positions_ptr); 
    void setDesiredJointVelocitiesPtr(
        Eigen::VectorXd* desired_joint_velocities_ptr);
    // void setDesiredPoses(std::vector<Eigen::VectorXd> * desired_poses); // Set the desired poses of the tasks.

    // std::vector<double> getPose(const Eigen::VectorXd * joint_positions); // Get the pose of the end-effector.
    // const Eigen::MatrixXd * getJacobianPtr(int task_id); // Get the Jacobian of a task
    // const Eigen::MatrixXd * getJacobianInversePtr(int task_id); // Get the inverse of Jacobian of a task

private:
    Eigen::VectorXd clampJointLimits(Eigen::VectorXd desired_joint_positions);

    int m_n_joints;
    std::vector<std::shared_ptr<Task>> m_tasks;
    std::vector<std::array<double,2>> m_joint_limits;
    uint32_t* m_integral_dt_ptr;
    Eigen::VectorXd* m_current_joint_positions_ptr;
    Eigen::VectorXd* m_desired_joint_positions_ptr;
    Eigen::VectorXd* m_desired_joint_velocities_ptr;
};

#endif // IK_SOLVER__IK_SOLVER_HPP_