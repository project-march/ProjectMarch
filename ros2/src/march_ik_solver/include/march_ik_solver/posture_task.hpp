/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef IK_SOLVER__POSTURE_TASK_HPP
#define IK_SOLVER__POSTURE_TASK_HPP

#include "march_ik_solver/task.hpp"

class PostureTask : public Task
{
public:
    PostureTask();

private:
    void computeCurrentTaskCoordinates() override;
    void computeCurrentTaskJacobian() override;

    const int LEFT_ANKLE_INDEX = 4;
    const int RIGHT_ANKLE_INDEX = 9;
};

#endif // IK_SOLVER__POSTURE_TASK_HPP