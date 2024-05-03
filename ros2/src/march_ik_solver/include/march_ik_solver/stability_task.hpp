/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef IK_SOLVER__STABILITY_TASK_HPP
#define IK_SOLVER__STABILITY_TASK_HPP

#include "march_ik_solver/task.hpp"

class StabilityTask : public Task
{
public:
    StabilityTask(const std::string& task_name, const std::string& reference_frame, const std::vector<int>& joint_indices,
    const unsigned int& workspace_dim, const unsigned int& configuration_dim, const float& dt);

    inline void setCurrentLinearAccelerationPtr(Eigen::Vector3d* current_linear_acceleration_ptr) { m_current_linear_acceleration_ptr = current_linear_acceleration_ptr; }
    inline void setCurrentStanceLegPtr(uint8_t* current_stance_leg_ptr) { m_current_stance_leg_ptr = current_stance_leg_ptr; }
    inline void setNextStanceLegPtr(uint8_t* next_stance_leg_ptr) { m_next_stance_leg_ptr = next_stance_leg_ptr; }

private:
    void computeCurrentTaskCoordinates() override;
    void computeCurrentTaskJacobian() override;

    double m_total_mass;
    Eigen::Vector3d* m_current_linear_acceleration_ptr;
    uint8_t* m_current_stance_leg_ptr;
    uint8_t* m_next_stance_leg_ptr;

    const int LEFT_FOOT_INDEX = 5;
    const int RIGHT_FOOT_INDEX = 10;
    const unsigned int TRANSLATION_SIZE = 3;
    const double FOOT_LENGTH = 0.32; // m
    const double FOOT_WIDTH = 0.15; // m
};

#endif  // IK_SOLVER__STABILITY_TASK_HPP