/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_ik_solver/stability_task.hpp"

#include "pinocchio/algorithm/center-of-mass.hpp"

StabilityTask::StabilityTask(const std::string& task_name, const std::string& reference_frame, const std::vector<int>& joint_indices,
    const unsigned int& workspace_dim, const unsigned int& configuration_dim, const float& dt)
    : Task(task_name, reference_frame, joint_indices, workspace_dim, configuration_dim, dt)
{
    m_total_mass = pinocchio::computeTotalMass(m_model);
}

void StabilityTask::computeCurrentTaskCoordinates()
{
    Eigen::VectorXd com_position;
    com_position.noalias() = m_current_world_to_base_orientation.transpose() * pinocchio::centerOfMass(m_model, *m_data);
    m_current_task = com_position.segment(0, 2);
}

void StabilityTask::computeCurrentTaskJacobian()
{
    Eigen::MatrixXd com_jacobian;
    com_jacobian.noalias() = m_current_world_to_base_orientation.transpose() * pinocchio::jacobianCenterOfMass(m_model, *m_data);
    m_jacobian = com_jacobian.topRows(2);
}