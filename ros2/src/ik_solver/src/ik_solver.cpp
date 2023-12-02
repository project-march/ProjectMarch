#include "ik_solver/ik_solver.hpp"

IKSolver::IKSolver()
{
    n_joints_ = 8;      // TODO: Load this from a YAML file
    configureTasks();
}

IKSolver::~IKSolver()
{
    delete &tasks_;
    // delete &desired_poses_;
}

Eigen::VectorXd IKSolver::solve(std::vector<Eigen::VectorXd> desired_poses)
{
    // Solve the IK problem
    Eigen::VectorXd joint_velocities = Eigen::VectorXd::Zero(n_joints_);
    // TODO: Uncomment code after MVP.
    // Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(n_joints_, n_joints_);

    // Calculate the joint velocities from the tasks
    for (long unsigned int i = 0; i < tasks_.size(); i++) // TODO: Zip the tasks and desired poses.
    {
        // const Eigen::MatrixXd * J_ptr = task.getJacobianPtr();
        // const Eigen::MatrixXd * J_inv_ptr = task.getJacobianInversePtr();
        // Eigen::VectorXd null_space_projection = (identity - *J_ptr * *J_inv_ptr) * joint_velocities;
        tasks_[i].setDesiredPose(&desired_poses[i]);
        joint_velocities += tasks_[i].solve();
    }

    return joint_velocities;
}

void IKSolver::configureTasks()
{
    // TODO: Load the tasks from a YAML file.
    Task task = {0, "motion", 6, 8};
    task.setGainP(1.0);
    task.setDampingCoefficient(1e-2);
    tasks_.push_back(task);
}

const Eigen::MatrixXd * IKSolver::getJacobianPtr(int task_id)
{
    // Get the Jacobian of a task
    return tasks_[task_id].getJacobianPtr();
}

const Eigen::MatrixXd * IKSolver::getJacobianInversePtr(int task_id)
{
    // Get the inverse of Jacobian of a task
    return tasks_[task_id].getJacobianInversePtr();
}