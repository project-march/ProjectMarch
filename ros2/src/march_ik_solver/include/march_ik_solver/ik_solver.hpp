#ifndef IK_SOLVER__IK_SOLVER_HPP_
#define IK_SOLVER__IK_SOLVER_HPP_

#pragma once
#include <string>
#include <unordered_map>
#include <vector>

#include "march_ik_solver/task.hpp"
#include "rclcpp/rclcpp.hpp"

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

    inline std::vector<double> getCurrentJointPositions() const { return std::vector<double>(m_current_joint_positions.data(), m_current_joint_positions.data() + m_current_joint_positions.size()); };
    inline std::vector<double> getCurrentJointVelocities() const { return std::vector<double>(m_current_joint_velocities.data(), m_current_joint_velocities.data() + m_current_joint_velocities.size()); };
    inline std::vector<double> getDesiredJointVelocities() const { return std::vector<double>(m_desired_joint_velocities.data(), m_desired_joint_velocities.data() + m_desired_joint_velocities.size()); };
    inline double getTasksError() const {
        double error = 0.0;
        for (const auto& task_name : m_task_names) {
            error += m_task_map.at(task_name)->getErrorNorm();
        }
        return error;
    };

    inline void setDt(const double& dt) { m_dt = dt; };
    inline void setTaskNames(const std::vector<std::string>& task_names) { m_task_names = task_names; };
    void setJointConfigurations(const std::vector<std::string>& joint_names,
        const std::vector<double>& joint_position_lower_limits, const std::vector<double>& joint_position_upper_limits,
        const std::vector<double>& joint_velocity_lower_limits, const std::vector<double>& joint_velocity_upper_limits);

private:
    Eigen::VectorXd clampJointLimits(Eigen::VectorXd desired_joint_positions);
    Eigen::VectorXd clampJointVelocities(Eigen::VectorXd desired_joint_velocities);
    inline double deg2rad(const double& deg) { return deg * M_PI / 180.0; };

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