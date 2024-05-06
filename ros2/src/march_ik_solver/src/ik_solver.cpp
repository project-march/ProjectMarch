/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_ik_solver/ik_solver.hpp"
#include "march_ik_solver/stability_task.hpp"
#include "march_ik_solver/motion_task.hpp"
#include "march_ik_solver/posture_task.hpp"

#include <algorithm>
#include <boost/algorithm/clamp.hpp>
#include <math.h>

IKSolver::IKSolver()
{
    m_dt = 1e-3; // s, 1 kHz control loop
    m_current_joint_positions = Eigen::VectorXd::Zero(0);
    m_current_joint_velocities = Eigen::VectorXd::Zero(0);
    m_desired_joint_velocities = Eigen::VectorXd::Zero(0);
    m_current_linear_acceleration = Eigen::Vector3d::Zero();
}

void IKSolver::createTask(
    std::unordered_map<std::string, std::vector<double>> task_gains_p,
    std::unordered_map<std::string, std::vector<double>> task_gains_d,
    std::unordered_map<std::string, std::vector<double>> task_gains_i,
    std::unordered_map<std::string, double> task_damp_coeffs,
    std::unordered_map<std::string, double> task_convergence_thresholds)
{
    // Create tasks
    m_task_map["motion"] = std::make_unique<MotionTask>();
    m_task_map["posture"] = std::make_unique<PostureTask>();

    // Create and set stability task
    std::unique_ptr<StabilityTask> stability_task = std::make_unique<StabilityTask>();
    stability_task->setCurrentLinearAccelerationPtr(&m_current_linear_acceleration);
    stability_task->setCurrentStanceLegPtr(&m_current_stance_leg);
    stability_task->setNextStanceLegPtr(&m_next_stance_leg);
    m_task_map["stability"] = std::move(stability_task);

    // Set task parameters
    for (const auto& task_name : m_task_names) {
        m_task_map.at(task_name)->configureIkSolverVariables();
        m_task_map.at(task_name)->setDt(m_dt);
        m_task_map.at(task_name)->setGainP(task_gains_p.at(task_name));
        m_task_map.at(task_name)->setGainD(task_gains_d.at(task_name));
        m_task_map.at(task_name)->setGainI(task_gains_i.at(task_name));
        m_task_map.at(task_name)->setDampingCoefficient(task_damp_coeffs.at(task_name));
        m_task_map.at(task_name)->setJointNamesPtr(&m_joint_names);
        m_task_map.at(task_name)->setCurrentJointPositionsPtr(&m_current_joint_positions);
        m_task_map.at(task_name)->setConvergenceThreshold(task_convergence_thresholds.at(task_name));
    }
}

Eigen::VectorXd IKSolver::solveInverseKinematics()
{
    m_desired_joint_velocities = Eigen::VectorXd::Zero(m_joint_names.size());
    for (const auto& task_name : m_task_names) {
        m_task_map.at(task_name)->computeCurrentTask();
        m_desired_joint_velocities.noalias() += m_task_map.at(task_name)->solveTask()
            + m_task_map.at(task_name)->getNullspaceProjection() * m_desired_joint_velocities;
    }
    // m_desired_joint_velocities = clampJointVelocities(m_desired_joint_velocities); // TODO: Do we need to clamp joint velocities in this manner?
    return m_desired_joint_velocities;
}

Eigen::VectorXd IKSolver::integrateJointVelocities()
{
    Eigen::VectorXd desired_joint_positions;
    desired_joint_positions.noalias() = m_current_joint_positions + m_desired_joint_velocities * m_dt;
    m_current_joint_positions = clampJointLimits(desired_joint_positions);
    return m_current_joint_positions;
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

void IKSolver::updateWorldToBaseOrientation(const double& w, const double& x, const double& y, const double& z)
{
    Eigen::Matrix3d current_world_to_base_orientation = Eigen::Quaterniond(w, x, y, z).toRotationMatrix();
    for (const auto& task_name : m_task_names) {
        m_task_map.at(task_name)->setCurrentWorldToBaseOrientation(current_world_to_base_orientation);
    }
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