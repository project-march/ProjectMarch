/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_ik_solver/posture_task.hpp"

#include <algorithm>
#include <functional>
#include <memory>

#include "pinocchio/algorithm/jacobian.hpp"
#include "math.h"

PostureTask::PostureTask() : Task()
{
    m_task_name = "posture";
    m_reference_frame = "base_link";
    m_joint_indices = {LEFT_ANKLE_INDEX, RIGHT_ANKLE_INDEX};
    setTaskM(m_joint_indices.size());
}

void PostureTask::computeCurrentTaskCoordinates()
{
    for (unsigned long int i = 0; i < m_joint_indices.size(); i++) {
        // Fixed-size template cannot be used with aliases
        Eigen::Matrix3d rotation_matrix = m_current_world_to_base_orientation.transpose() * m_data->oMi[m_joint_indices[i]].rotation();
        m_current_task(i) = rotation_matrix.eulerAngles(ROTATION_YAW_INDEX, ROTATION_PITCH_INDEX, ROTATION_ROLL_INDEX).y();
    }
}

void PostureTask::computeCurrentTaskJacobian()
{
    for (unsigned long int i = 0; i < m_joint_indices.size(); i++) {
        // Fixed-size template cannot be used with aliases
        pinocchio::Data::Matrix6x jacobian_joint(SE3_SIZE, m_model.nv);
        jacobian_joint.setZero();

        pinocchio::computeJointJacobian(m_model, *m_data, *m_current_joint_positions_ptr, m_joint_indices[i], jacobian_joint);
        Eigen::MatrixXd jacobian_orientation_matrix = m_current_world_to_base_orientation.transpose() * jacobian_joint.bottomRows(EUCLIDEAN_SIZE);
        m_jacobian.block(i, 0, 1, m_task_n) = jacobian_orientation_matrix.row(ROTATION_PITCH_INDEX);
    }
}