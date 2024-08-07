/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_ik_solver/task.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include <algorithm>
#include <functional>
#include <memory>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "math.h"

Task::Task()
{
    m_current_world_to_base_orientation = Eigen::Matrix3d::Identity();

    // Load URDF model
    std::string urdf_file_path = ament_index_cpp::get_package_share_directory("march_description") + "/urdf/march9/march9.urdf";
    pinocchio::urdf::buildModel(urdf_file_path, m_model);
    m_data = std::make_unique<pinocchio::Data>(m_model);
    m_task_n = m_model.nv;
}

Task::~Task()
{
    m_joint_names_ptr = new std::vector<std::string>;
    m_current_joint_positions_ptr = new Eigen::VectorXd;

    delete m_joint_names_ptr;
    delete m_current_joint_positions_ptr;

    m_joint_names_ptr = NULL;
    m_current_joint_positions_ptr = NULL;
}

void Task::setTaskName(const std::string& task_name)
{
    m_task_name = task_name;
}

void Task::setTaskM(const unsigned int& task_m)
{
    m_task_m = task_m;
    
    // Initialize gains as diagonal matrices
    m_gain_p = Eigen::MatrixXd::Zero(task_m, task_m);
    m_gain_d = Eigen::MatrixXd::Zero(task_m, task_m);
    m_gain_i = Eigen::MatrixXd::Zero(task_m, task_m);
    for (unsigned int i = 0; i < task_m; i++) {
        m_gain_p.diagonal()[i] = 1.0;
        m_gain_d.diagonal()[i] = 0.0;
        m_gain_i.diagonal()[i] = 0.0;
    }
}

void Task::setTaskN(const unsigned int& task_n)
{
    m_task_n = task_n;

    // Initialize damping identity matrix for singularity-robustness
    m_damping_identity = Eigen::MatrixXd::Identity(m_task_n, m_task_n);
}

void Task::setGainP(const std::vector<double>& gain_p)
{
    m_nonzero_gain_p_indices.clear();
    for (unsigned long int i = 0; i < gain_p.size(); i++) {
        m_gain_p.diagonal()[i] = gain_p[i];
        if (gain_p[i] > 0.0) {
            m_nonzero_gain_p_indices.push_back(i);
        }
    }
}

void Task::setGainD(const std::vector<double>& gain_d)
{
    for (unsigned long int i = 0; i < gain_d.size(); i++) {
        m_gain_d.diagonal()[i] = gain_d[i];
    }
}

void Task::setGainI(const std::vector<double>& gain_i)
{
    for (unsigned long int i = 0; i < gain_i.size(); i++) {
        m_gain_i.diagonal()[i] = gain_i[i];
    }
}

void Task::setDampingCoefficient(const double& damping_coefficient)
{
    m_damping_coefficient = damping_coefficient;
    m_damping_identity.noalias() = damping_coefficient * m_damping_identity;
}

Eigen::VectorXd Task::solveTask()
{
    Eigen::VectorXd error = calculateError();
    Eigen::VectorXd joint_velocities = Eigen::VectorXd::Zero(m_model.nv);
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> jacobian_decomposition(m_jacobian);
    jacobian_decomposition.setThreshold(m_damping_coefficient);
    joint_velocities.noalias() = jacobian_decomposition.solve(error);
    return joint_velocities;
}

Eigen::VectorXd Task::calculateError()
{
    Eigen::VectorXd error, pid_error;
    error.noalias() = m_desired_task - m_current_task;
    
    // Calculate relevant error norms
    m_error_norm = 0.0;
    for (const auto& nonzero_gain_p_idx : m_nonzero_gain_p_indices) {
        m_error_norm += pow(error(nonzero_gain_p_idx), 2);
    }
    m_error_norm = sqrt(m_error_norm);

    // Calculate PID error
    pid_error.noalias() = m_gain_p * error + calculateIntegralError(error) + calculateDerivativeError(error);

    m_previous_error = error;
    return pid_error;
}

Eigen::VectorXd Task::calculateIntegralError(const Eigen::VectorXd& error)
{
    Eigen::VectorXd integral_error;
    m_integral_error.noalias() = m_integral_error + error * m_dt;
    integral_error.noalias() = m_gain_i * m_integral_error;
    return integral_error;
}

Eigen::VectorXd Task::calculateDerivativeError(const Eigen::VectorXd& error)
{
    Eigen::VectorXd derivative_error;
    derivative_error.noalias() = m_gain_d * ((error - m_previous_error) / m_dt);
    return derivative_error;
}

void Task::computeCurrentTask()
{
    // Update joint positions in data
    pinocchio::forwardKinematics(m_model, *m_data, *m_current_joint_positions_ptr);

    // Compute current task coordinates and Jacobian
    computeCurrentTaskCoordinates();
    computeCurrentTaskJacobian();
    computeJacobianInverse();
}

void Task::configureIkSolverVariables()
{
    // Configure ik solver variables
    m_current_task = Eigen::VectorXd::Zero(m_task_m);
    m_jacobian = Eigen::MatrixXd::Zero(m_task_m, m_task_n);
    m_jacobian_inverse = Eigen::MatrixXd::Zero(m_task_n, m_task_m);
    m_previous_error = Eigen::VectorXd::Zero(m_task_m);
    m_integral_error = Eigen::VectorXd::Zero(m_task_m);
}

void Task::computeJacobianInverse()
{
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> jacobian_decomposition(m_jacobian);
    jacobian_decomposition.setThreshold(m_damping_coefficient);
    m_jacobian_inverse.noalias() = jacobian_decomposition.pseudoInverse();
}

double Task::getErrorNorm() const
{
    return m_error_norm;
}

std::vector<double> Task::getError() const
{
    return std::vector<double>(m_previous_error.data(), m_previous_error.data() + m_previous_error.size());
}

march_shared_msgs::msg::IksTaskStatus Task::getTaskStatus() const
{
    march_shared_msgs::msg::IksTaskStatus task_status;
    task_status.task_name = m_task_name;
    task_status.error_norm = m_error_norm;
    task_status.error = std::vector<double>(m_previous_error.data(), m_previous_error.data() + m_previous_error.size());
    return task_status;
}

Eigen::MatrixXd Task::getNullspaceProjection() const
{
    Eigen::MatrixXd nullspace_projection;
    nullspace_projection.noalias() = Eigen::MatrixXd::Identity(m_task_n, m_task_n) - m_jacobian_inverse * m_jacobian;
    return nullspace_projection;
}

void Task::computeCurrentTaskCoordinates()
{
    // Implement in derived classes
}

void Task::computeCurrentTaskJacobian()
{
    // Implement in derived classes
}