#ifndef IK_SOLVER__IK_SOLVER_HPP_
#define IK_SOLVER__IK_SOLVER_HPP_

#include <algorithm>
#include <vector>
#include <string>

#include "ik_solver/task.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

class IKSolver
{
    public:
        IKSolver();
        Eigen::VectorXf getJointConfig();
        Eigen::MatrixXf getJacobian();
        Eigen::MatrixXf getJacobianInverse();

    private:
        // std::vector<Task> tasks_;   // A stack of tasks, the order of which is the priority of the tasks.
        Task task_;                     // A single task
        Eigen::VectorXf joint_config_;  // The joint configuration of the robot
};

#endif  // IK_SOLVER__IK_SOLVER_HPP_