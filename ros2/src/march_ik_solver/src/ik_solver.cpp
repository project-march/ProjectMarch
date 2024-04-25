#include "march_ik_solver/ik_solver.hpp"

#include <algorithm>
#include <boost/algorithm/clamp.hpp>
#include <math.h>

void IKSolver::createTask(const std::string& name, const std::string reference_frame,
    const std::vector<std::string>& node_names, const unsigned int& workspace_dim,
    const unsigned int& configuration_dim, const std::vector<double>& gain_p, const std::vector<double>& gain_d,
    const std::vector<double>& gain_i, const double& damping_coefficient)
{
    m_task_map[name] = std::make_unique<Task>(name, reference_frame, workspace_dim, configuration_dim, m_dt);
    m_task_map[name]->setNodeNames(node_names);
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
        m_task_map.at(task_name)->requestCurrentTask();
        m_desired_joint_velocities.noalias() += m_task_map.at(task_name)->solveTask();
            // + m_task_map.at(task_name)->getNullspaceProjection() * m_desired_joint_velocities;
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

std::vector<double> IKSolver::getCurrentJointPositions() const
{
    return std::vector<double>(
        m_current_joint_positions.data(), m_current_joint_positions.data() + m_current_joint_positions.size());
}

std::vector<double> IKSolver::getCurrentJointVelocities() const
{
    return std::vector<double>(
        m_current_joint_velocities.data(), m_current_joint_velocities.data() + m_current_joint_velocities.size());
}

std::vector<double> IKSolver::getDesiredJointVelocities() const
{
    return std::vector<double>(
        m_desired_joint_velocities.data(), m_desired_joint_velocities.data() + m_desired_joint_velocities.size());
}

double IKSolver::getTasksError() const
{
    Eigen::VectorXd tasks_error = Eigen::VectorXd::Zero(m_task_names.size());

    for (long unsigned int i = 0; i < m_task_names.size(); i++) {
        tasks_error(i) = m_task_map.at(m_task_names[i])->getErrorNorm();
    }

    return tasks_error.norm();
}

void IKSolver::setDt(const double& dt)
{
    m_dt = dt;
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

    for (long unsigned int i = 0; i < joint_names.size(); i++) {
        m_joint_position_limits.push_back({ deg2rad(joint_position_lower_limits[i]), deg2rad(joint_position_upper_limits[i]) });
        m_joint_velocity_limits.push_back({ deg2rad(joint_velocity_lower_limits[i]), deg2rad(joint_velocity_upper_limits[i]) });
    }
}

void IKSolver::setTaskNames(const std::vector<std::string>& task_names)
{
    m_task_names = task_names;
}