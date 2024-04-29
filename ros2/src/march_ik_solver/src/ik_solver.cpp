#include "march_ik_solver/ik_solver.hpp"

#include <algorithm>
#include <boost/algorithm/clamp.hpp>
#include <math.h>

void IKSolver::createTask(const std::string& name, const std::string reference_frame,
    const std::vector<int>& joint_indices, const unsigned int& workspace_dim,
    const unsigned int& configuration_dim, const std::vector<double>& gain_p, const std::vector<double>& gain_d,
    const std::vector<double>& gain_i, const double& damping_coefficient)
{
    m_task_map[name] = std::make_unique<Task>(name, reference_frame, joint_indices, workspace_dim, configuration_dim, m_dt);
    m_task_map[name]->setGainP(gain_p);
    m_task_map[name]->setGainD(gain_d);
    m_task_map[name]->setGainI(gain_i);
    m_task_map[name]->setDampingCoefficient(damping_coefficient);
    m_task_map[name]->setJointNamesPtr(&m_joint_names);
    m_task_map[name]->setCurrentJointPositionsPtr(&m_current_joint_positions);
}

void IKSolver::updateDesiredTasks(const std::unordered_map<std::string, Eigen::VectorXd>& desired_tasks)
{
    for (const auto& task_name : m_task_names) {
        m_task_map.at(task_name)->setDesiredTask(desired_tasks.at(task_name));
    }
}

void IKSolver::updateCurrentJointState(
    const std::vector<double>& current_joint_positions, const std::vector<double>& current_joint_velocities)
{
    m_current_joint_positions
        = Eigen::Map<const Eigen::VectorXd>(current_joint_positions.data(), current_joint_positions.size());
    m_current_joint_velocities
        = Eigen::Map<const Eigen::VectorXd>(current_joint_velocities.data(), current_joint_velocities.size());
}

Eigen::VectorXd IKSolver::solveInverseKinematics()
{
    m_desired_joint_velocities = Eigen::VectorXd::Zero(m_joint_names.size());
    for (const auto& task_name : m_task_names) {
        m_task_map.at(task_name)->computeCurrentTask();
        m_desired_joint_velocities.noalias() += m_task_map.at(task_name)->solveTask()
            + m_task_map.at(task_name)->getNullspaceProjection() * m_desired_joint_velocities;
    }
    m_desired_joint_velocities = clampJointVelocities(m_desired_joint_velocities);
    return m_desired_joint_velocities;
}

Eigen::VectorXd IKSolver::integrateJointVelocities()
{
    Eigen::VectorXd desired_joint_positions;
    desired_joint_positions.noalias() = m_current_joint_positions + m_desired_joint_velocities * m_dt;
    m_current_joint_positions = clampJointLimits(desired_joint_positions);
    return m_current_joint_positions;
}

Eigen::VectorXd IKSolver::clampJointLimits(Eigen::VectorXd desired_joint_positions)
{
    Eigen::VectorXd limited_joint_positions = Eigen::VectorXd::Zero(m_joint_names.size());
    for (long unsigned int i = 0; i < m_joint_position_limits.size(); i++) {
        limited_joint_positions(i) = boost::algorithm::clamp(
            desired_joint_positions(i), 
            m_joint_position_limits[i][LOWER_JOINT_LIMIT], 
            m_joint_position_limits[i][UPPER_JOINT_LIMIT]);
    }
    return limited_joint_positions;
}

Eigen::VectorXd IKSolver::clampJointVelocities(Eigen::VectorXd desired_joint_velocities)
{
    Eigen::VectorXd limited_joint_velocities = Eigen::VectorXd::Zero(m_joint_names.size());
    for (long unsigned int i = 0; i < m_joint_position_limits.size(); i++) {
        limited_joint_velocities(i) = boost::algorithm::clamp(
            desired_joint_velocities(i), 
            m_joint_velocity_limits[i][LOWER_JOINT_LIMIT], 
            m_joint_velocity_limits[i][UPPER_JOINT_LIMIT]);
    }
    return limited_joint_velocities;
}

void IKSolver::setJointConfigurations(const std::vector<std::string>& joint_names,
    const std::vector<double>& joint_position_lower_limits, const std::vector<double>& joint_position_upper_limits,
    const std::vector<double>& joint_velocity_lower_limits, const std::vector<double>& joint_velocity_upper_limits)
{
    m_joint_names = joint_names;

    if (joint_position_lower_limits.size() != joint_position_upper_limits.size()) {
        RCLCPP_ERROR(rclcpp::get_logger("ik_solver"),
            "IKSolver::setJointLimits(): lower_joint_limits.size() != upper_joint_limits.size()");
        return;
    }

    double multiplier = 1.0;
    for (long unsigned int i = 0; i < joint_names.size(); i++) {
        m_joint_position_limits.push_back({ deg2rad(joint_position_lower_limits[i]), deg2rad(joint_position_upper_limits[i]) });
        m_joint_velocity_limits.push_back({ multiplier * deg2rad(joint_velocity_lower_limits[i]), multiplier * deg2rad(joint_velocity_upper_limits[i]) });
    }
}