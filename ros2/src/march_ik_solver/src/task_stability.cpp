/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_ik_solver/task_stability.hpp"

#include "pinocchio/algorithm/center-of-mass.hpp"

TaskStability::TaskStability(const std::string& task_name, const std::string& reference_frame, const std::vector<int>& joint_indices,
    const unsigned int& workspace_dim, const unsigned int& configuration_dim, const float& dt)
    : Task(task_name, reference_frame, joint_indices, workspace_dim, configuration_dim, dt)
{
    m_total_mass = pinocchio::computeTotalMass(m_model);
}

void TaskStability::computeCurrentTaskCoordinates()
{
    m_current_task = pinocchio::centerOfMass(m_model, *m_data).segment(0, 2);
}

void TaskStability::computeCurrentTaskJacobian()
{
   m_jacobian = pinocchio::jacobianCenterOfMass(m_model, *m_data).topRows(2);
}