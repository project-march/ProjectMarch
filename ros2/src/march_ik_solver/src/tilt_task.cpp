/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_ik_solver/tilt_task.hpp"

#include <algorithm>
#include <functional>
#include <memory>

#include "pinocchio/algorithm/jacobian.hpp"
#include "math.h"

TiltTask::TiltTask() : Task()
{
    m_task_name = "tilt";
    m_reference_frame = "base_link";
    m_joint_indices = {LEFT_HIP_FE_INDEX, RIGHT_HIP_FE_INDEX};
    setTaskM(m_joint_indices.size());
}

void TiltTask::computeCurrentTaskCoordinates()
{
    for (unsigned long int i = 0; i < m_joint_indices.size(); i++) {
        m_current_task(i) = (*m_current_joint_positions_ptr)[m_joint_indices[i] - 1];
    }
}

void TiltTask::computeCurrentTaskJacobian()
{
    m_jacobian = Eigen::MatrixXd::Zero(m_task_m, m_model.nv);
    for (unsigned long int i = 0; i < m_joint_indices.size(); i++) {
        m_jacobian(i, m_joint_indices[i] - 1) = 1.0;
    }
}