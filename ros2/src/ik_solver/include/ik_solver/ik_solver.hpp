#ifndef IK_SOLVER__IK_SOLVER_HPP_
#define IK_SOLVER__IK_SOLVER_HPP_

#include <algorithm>
#include <vector>
#include <string>

#include "ik_solver/task.hpp"
#include "ik_solver/ik_solver_command.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

class IKSolver
{
    public:
        IKSolver();                     // Default constructor.
        ~IKSolver();                    // Default destructor.
        Eigen::VectorXd solve(std::vector<Eigen::VectorXd> desired_poses); // Solve the IK problem.

        const Eigen::MatrixXd * getJacobianPtr(int task_id); // Get the Jacobian of a task
        const Eigen::MatrixXd * getJacobianInversePtr(int task_id); // Get the inverse of Jacobian of a task

    private:
        
        void configureTasks();                      // Configure the tasks

        std::vector<Task> tasks_;           // A stack of tasks, the order of which is the priority of the tasks.
        int n_joints_;                      // Number of joints. TODO: Load this from a YAML file?
};

#endif  // IK_SOLVER__IK_SOLVER_HPP_