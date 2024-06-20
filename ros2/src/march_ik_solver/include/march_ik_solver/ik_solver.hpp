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

    IKSolver();
    ~IKSolver() = default;

    void createTask(
        std::unordered_map<std::string, std::vector<double>> task_gains_p,
        std::unordered_map<std::string, std::vector<double>> task_gains_d,
        std::unordered_map<std::string, std::vector<double>> task_gains_i,
        std::unordered_map<std::string, double> task_damp_coeffs,
        std::unordered_map<std::string, double> task_convergence_thresholds,
        std::unordered_map<std::string, double> task_weights);
    Eigen::VectorXd solveInverseKinematics();
    Eigen::VectorXd integrateJointVelocities();
    Eigen::VectorXd applyJointVelocityLimits(
        const double& dt,
        const Eigen::VectorXd& desired_joint_positions, 
        const Eigen::VectorXd& current_joint_positions) const;
    bool areTasksConverged();

    void updateDesiredTasks(const std::unordered_map<std::string, Eigen::VectorXd>& desired_tasks);
    void updateCurrentJointState(
        const std::vector<double>& current_joint_positions, const std::vector<double>& current_joint_velocities);
    void updateCurrentWorldToBaseOrientation(const Eigen::Matrix3d& current_world_to_base_orientation);
    inline void updateCurrentStanceLeg(const uint8_t& current_stance_leg) { m_current_stance_leg = current_stance_leg; };
    inline void updateNextStanceLeg(const uint8_t& next_stance_leg) { m_next_stance_leg = next_stance_leg; };
    inline void updateCurrentLinearAcceleration(const Eigen::Vector3d& current_linear_acceleration) { m_current_linear_acceleration = current_linear_acceleration; };

    inline std::vector<std::string> getTaskNames() const { return m_task_names; };
    inline std::vector<double> getCurrentJointPositions() const { return std::vector<double>(m_current_joint_positions.data(), m_current_joint_positions.data() + m_current_joint_positions.size()); };
    inline std::vector<double> getCurrentJointVelocities() const { return std::vector<double>(m_current_joint_velocities.data(), m_current_joint_velocities.data() + m_current_joint_velocities.size()); };
    inline std::vector<double> getDesiredJointVelocities() const { return std::vector<double>(m_desired_joint_velocities.data(), m_desired_joint_velocities.data() + m_desired_joint_velocities.size()); };
    inline double getTasksError() const {
        double error = 0.0;
        for (const auto& task_name : m_task_names) {
            error += m_task_map.at(task_name)->getErrorNorm();
        }
        return error / m_task_names.size();
    };
    inline march_shared_msgs::msg::IksStatus getIKStatus() const {
        march_shared_msgs::msg::IksStatus iks_status;
        for (const auto& task_name : m_task_names) {
            iks_status.tasks.push_back(m_task_map.at(task_name)->getTaskStatus());
        }
        return iks_status;
    }
    inline bool isPrioritizedTaskConverged() const { return m_task_map.at(m_task_names.back())->isConverged(); };

    inline void setDt(const double& dt) { m_dt = dt; };
    inline void setVelocityLimitMultiplier(const double& velocity_limit_multiplier) { m_velocity_limit_multiplier = velocity_limit_multiplier; };
    inline void setTaskNames(const std::vector<std::string>& task_names) { m_task_names = task_names; };
    void setJointConfigurations(const std::vector<std::string>& joint_names,
        const std::vector<double>& joint_position_lower_limits, const std::vector<double>& joint_position_upper_limits,
        const std::vector<double>& joint_velocity_lower_limits, const std::vector<double>& joint_velocity_upper_limits,
        const double& velocity_limit_multiplier);

private:
    Eigen::VectorXd clampJointLimits(Eigen::VectorXd desired_joint_positions);
    Eigen::VectorXd clampJointVelocities(Eigen::VectorXd desired_joint_velocities);
    inline double deg2rad(const double& deg) { return deg * M_PI / 180.0; };

    const unsigned int LOWER_JOINT_LIMIT = 0;
    const unsigned int UPPER_JOINT_LIMIT = 1;

    double m_dt;
    double m_velocity_limit_multiplier;
    std::vector<std::string> m_task_names;
    std::unordered_map<std::string, Task::UniquePtr> m_task_map;

    uint8_t m_current_stance_leg;
    uint8_t m_next_stance_leg;
    Eigen::VectorXd m_current_joint_positions;
    Eigen::VectorXd m_current_joint_velocities;
    Eigen::VectorXd m_desired_joint_velocities;
    Eigen::Vector3d m_current_linear_acceleration;

    std::vector<std::string> m_joint_names;
    std::vector<JointLimit> m_joint_position_limits;
    std::vector<JointLimit> m_joint_velocity_limits;
};

#endif // IK_SOLVER__IK_SOLVER_HPP_