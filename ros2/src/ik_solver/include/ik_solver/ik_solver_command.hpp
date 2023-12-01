#ifndef IK_SOLVER__IK_SOLVER_COMMAND_HPP_
#define IK_SOLVER__IK_SOLVER_COMMAND_HPP_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

struct IKSolverCommand  // TODO: Move this to a separate file
{
    Eigen::VectorXf desired_pose;
    Eigen::VectorXf current_pose;
    Eigen::VectorXf joint_config;
};

#endif  // IK_SOLVER__IK_SOLVER_COMMAND_HPP_