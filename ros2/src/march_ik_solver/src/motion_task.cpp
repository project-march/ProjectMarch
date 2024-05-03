/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_ik_solver/motion_task.hpp"

#include <algorithm>
#include <functional>
#include <memory>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "math.h"

MotionTask::MotionTask() : Task()
{
    m_task_name = "motion";
    m_reference_frame = "base_link";
    m_joint_indices = {LEFT_ANKLE_INDEX, RIGHT_ANKLE_INDEX};
    setTaskM(m_joint_indices.size() * EUCLIDEAN_SIZE);
}

void MotionTask::computeCurrentTaskCoordinates()
{
    for (unsigned long int i = 0; i < m_joint_indices.size(); i++) {
        // Fixed-size template cannot be used with aliases
        m_current_task.segment<3>(EUCLIDEAN_SIZE * i) = m_data->oMi[m_joint_indices[i]].translation();
    }
}

void MotionTask::computeCurrentTaskJacobian()
{
    for (unsigned long int i = 0; i < m_joint_indices.size(); i++) {
        // Fixed-size template cannot be used with aliases
        pinocchio::Data::Matrix6x jacobian_joint(SE3_SIZE, m_model.nv);
        jacobian_joint.setZero();

        pinocchio::computeJointJacobian(m_model, *m_data, *m_current_joint_positions_ptr, m_joint_indices[i], jacobian_joint);
        m_jacobian.block(EUCLIDEAN_SIZE * i, 0, EUCLIDEAN_SIZE, m_model.nv) = jacobian_joint.topRows(EUCLIDEAN_SIZE);
    }
}