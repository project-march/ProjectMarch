/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef IK_SOLVER__TILT_TASK_HPP
#define IK_SOLVER__TILT_TASK_HPP

#include "march_ik_solver/task.hpp"

class TiltTask : public Task
{
public:
    TiltTask();

private:
    void computeCurrentTaskCoordinates() override;
    void computeCurrentTaskJacobian() override;

    const int LEFT_HIP_FE_INDEX = 2;
    const int RIGHT_HIP_FE_INDEX = 7;
};

#endif // IK_SOLVER__TILT_TASK_HPP