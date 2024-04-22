/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef IK_SOLVER__IK_SOLVER_HPP_
#define IK_SOLVER__IK_SOLVER_HPP_

#pragma once
#include <string>
#include <unordered_map>
#include <vector>

#include "march_ik_solver/task.hpp"
#include "rclcpp/rclcpp.hpp"
#include "march_shared_msgs/msg/iks_status.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

class IKSolver {
public:
    typedef std::array<double, 2> JointLimit;

    IKSolver() = default;
    ~IKSolver() = default;

    void createTask(const std::string& name, const std::string reference_frame,
        const std::vector<std::string>& node_names, const unsigned int& workspace_dim,
        const unsigned int& configuration_dim, const std::vector<double>& gain_p, const std::vector<double>& gain_d,
        const std::vector<double>& gain_i, const double& damping_coefficient);
    void updateDesiredTasks(const std::unordered_map<std::string, Eigen::VectorXd>& desired_tasks);
    void updateCurrentJointState(
        const std::vector<double>& current_joint_positions, const std::vector<double>& current_joint_velocities);
    Eigen::VectorXd solveInverseKinematics();
    Eigen::VectorXd integrateJointVelocities();

    std::vector<double> getCurrentJointPositions() const;
    std::vector<double> getCurrentJointVelocities() const;
    std::vector<double> getDesiredJointVelocities() const;
    double getTasksError() const;
    march_shared_msgs::msg::IksStatus getIKStatus() const;

    void setDt(const double& dt);
    void setJointConfigurations(const std::vector<std::string>& joint_names,
        const std::vector<double>& joint_position_lower_limits, const std::vector<double>& joint_position_upper_limits,
        const std::vector<double>& joint_velocity_lower_limits, const std::vector<double>& joint_velocity_upper_limits);
    void setTaskNames(const std::vector<std::string>& task_names);

private:
    Eigen::VectorXd clampJointLimits(Eigen::VectorXd desired_joint_positions);
    Eigen::VectorXd clampJointVelocities(Eigen::VectorXd desired_joint_velocities);

    const unsigned int LOWER_JOINT_LIMIT = 0;
    const unsigned int UPPER_JOINT_LIMIT = 1;

    double m_dt;
    std::vector<std::string> m_task_names;
    std::unordered_map<std::string, Task::UniquePtr> m_task_map;
    Eigen::VectorXd m_current_joint_positions;
    Eigen::VectorXd m_current_joint_velocities;
    Eigen::VectorXd m_desired_joint_velocities;

    std::vector<std::string> m_joint_names;
    std::vector<JointLimit> m_joint_position_limits;
    std::vector<JointLimit> m_joint_velocity_limits;
};

#endif // IK_SOLVER__IK_SOLVER_HPP_