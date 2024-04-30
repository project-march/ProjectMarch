/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef IK_SOLVER__TASK_STABILITY_HPP
#define IK_SOLVER__TASK_STABILITY_HPP

#include "march_ik_solver/task.hpp"

class TaskStability : public Task
{
public:
    TaskStability(const std::string& task_name, const std::string& reference_frame, const std::vector<int>& joint_indices,
    const unsigned int& workspace_dim, const unsigned int& configuration_dim, const float& dt);

private:
    void computeCurrentTaskCoordinates() override;
    void computeCurrentTaskJacobian() override;

    double m_total_mass; 

    const unsigned int TRANSLATION_SIZE = 3;   
};

#endif  // IK_SOLVER__TASK_STABILITY_HPP