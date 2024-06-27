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

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pinocchio/parsers/urdf.hpp"

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
    std::unordered_map<std::string, double> task_convergence_thresholds,
    std::unordered_map<std::string, double> task_weights)
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
        m_task_map.at(task_name)->setWeight(task_weights.at(task_name));
    }
}

Eigen::VectorXd IKSolver::solveInverseKinematics()
{
    m_desired_joint_velocities = Eigen::VectorXd::Zero(m_joint_names.size());
    double previous_task_weight = 0.0;
    for (const auto& task_name : m_task_names) {
        m_task_map.at(task_name)->computeCurrentTask();
        m_desired_joint_velocities.noalias() = m_task_map.at(task_name)->solveTask()
            + previous_task_weight * (m_task_map.at(task_name)->getNullspaceProjection() * m_desired_joint_velocities);
        previous_task_weight = m_task_map.at(task_name)->getWeight();
    }
    return m_desired_joint_velocities;
}

Eigen::VectorXd IKSolver::integrateJointVelocities()
{
    Eigen::VectorXd desired_joint_positions;
    desired_joint_positions.noalias() = m_current_joint_positions + m_desired_joint_velocities * m_dt;
    m_current_joint_positions = clampJointLimits(desired_joint_positions);
    return m_current_joint_positions;
}

Eigen::VectorXd IKSolver::applyJointVelocityLimits(const double& dt,
    const Eigen::VectorXd& desired_joint_positions, 
    const Eigen::VectorXd& current_joint_positions) const
{
    Eigen::VectorXd joint_velocities = Eigen::VectorXd::Zero(m_joint_names.size());
    joint_velocities.noalias() = (desired_joint_positions - current_joint_positions) / dt;
    for (long unsigned int i = 0; i < m_joint_names.size(); i++) {
        joint_velocities(i) = boost::algorithm::clamp(
            joint_velocities(i), 
            m_joint_velocity_limits[i][LOWER_JOINT_LIMIT], 
            m_joint_velocity_limits[i][UPPER_JOINT_LIMIT]);
    }
    return joint_velocities * dt + current_joint_positions;
}

bool IKSolver::areTasksConverged()
{
    for (const auto& task_name : m_task_names) {
        if (!m_task_map.at(task_name)->isConverged()) {
            return false;
        }
    }
    return true;
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
    Eigen::Matrix3d current_world_to_base_orientation = Eigen::Quaterniond(w, x, y, z).normalized().toRotationMatrix();
    for (const auto& task_name : m_task_names) {
        m_task_map.at(task_name)->setCurrentWorldToBaseOrientation(current_world_to_base_orientation);
    }
}

void IKSolver::configurePinocchioModel()
{
    // Load URDF model
    std::string urdf_file_path = ament_index_cpp::get_package_share_directory("march_description") + "/urdf/march9/march9.urdf";
    pinocchio::urdf::buildModel(urdf_file_path, m_pinocchio_model);
    m_pinocchio_data = std::make_unique<pinocchio::Data>(m_pinocchio_model);
}

void IKSolver::updatePinocchioModel(const Eigen::VectorXd& joint_positions)
{
    pinocchio::forwardKinematics(m_pinocchio_model, *m_pinocchio_data, joint_positions);
}

std::vector<Eigen::Vector3d> IKSolver::getEndEffectorPositions() const
{
    const int LEFT_FOOT_ID = 4;
    const int RIGHT_FOOT_ID = 9;
    int end_effector_ids[] = { LEFT_FOOT_ID, RIGHT_FOOT_ID };
    std::vector<Eigen::Vector3d> end_effector_positions;

    for (const auto& end_effector_id : end_effector_ids) {
        Eigen::Vector3d end_effector_position = m_pinocchio_data->oMi[end_effector_id].translation();
        end_effector_positions.push_back(end_effector_position);
    }

    return end_effector_positions;
}

std::vector<Eigen::Matrix3d> IKSolver::getEndEffectorOrientations() const
{
    const int LEFT_FOOT_ID = 4;
    const int RIGHT_FOOT_ID = 9;
    int end_effector_ids[] = { LEFT_FOOT_ID, RIGHT_FOOT_ID };
    std::vector<Eigen::Matrix3d> end_effector_orientations;

    for (const auto& end_effector_id : end_effector_ids) {
        Eigen::Matrix3d end_effector_orientation = m_pinocchio_data->oMi[end_effector_id].rotation();
        end_effector_orientations.push_back(end_effector_orientation);
    }

    return end_effector_orientations;
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
    const std::vector<double>& joint_velocity_lower_limits, const std::vector<double>& joint_velocity_upper_limits,
    const double& joint_velocity_multiplier)
{
    m_joint_names = joint_names;

    if (joint_position_lower_limits.size() != joint_position_upper_limits.size()) {
        RCLCPP_ERROR(rclcpp::get_logger("ik_solver"),
            "IKSolver::setJointLimits(): lower_joint_limits.size() != upper_joint_limits.size()");
        return;
    }

    for (long unsigned int i = 0; i < joint_names.size(); i++) {
        m_joint_position_limits.push_back({ 
            deg2rad(joint_position_lower_limits[i]), 
            deg2rad(joint_position_upper_limits[i]) });
        m_joint_velocity_limits.push_back({ 
            joint_velocity_multiplier * deg2rad(joint_velocity_lower_limits[i]), 
            joint_velocity_multiplier * deg2rad(joint_velocity_upper_limits[i]) });
    }
}