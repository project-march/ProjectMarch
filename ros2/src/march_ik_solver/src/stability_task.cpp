/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_ik_solver/stability_task.hpp"

#include "pinocchio/algorithm/center-of-mass.hpp"

StabilityTask::StabilityTask() : Task()
{
    m_task_name = "stability";
    m_reference_frame = "world";
    m_total_mass = pinocchio::computeTotalMass(m_model);
    setTaskM(STABILITY_COORDINATE_SIZE);
}

void StabilityTask::computeCurrentTaskCoordinates()
{
    Eigen::VectorXd com_position;
    com_position.noalias() = m_current_world_to_base_orientation.transpose() * pinocchio::centerOfMass(m_model, *m_data);
    m_current_task = com_position.segment(0, STABILITY_COORDINATE_SIZE);
}

void StabilityTask::computeCurrentTaskJacobian()
{
    Eigen::MatrixXd com_jacobian;
    com_jacobian.noalias() = m_current_world_to_base_orientation.transpose() * pinocchio::jacobianCenterOfMass(m_model, *m_data);
    m_jacobian = com_jacobian.topRows(STABILITY_COORDINATE_SIZE);
}