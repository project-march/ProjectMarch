#include "march_ik_solver/ik_solver.hpp"

IKSolver::IKSolver()
{
    n_joints_ = 8; // TODO: Load this from a YAML file
    dt_ = 0.001; // TODO: Load this from a YAML file
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
    Eigen::VectorXd desired_joint_velocities = Eigen::VectorXd::Zero(n_joints_);
    // TODO: Uncomment code after MVP.
    // Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(n_joints_, n_joints_);

    // Calculate the joint velocities from the tasks
    for (long unsigned int i = 0; i < tasks_.size(); i++) // TODO: Zip the tasks and desired poses.
    {
        // const Eigen::MatrixXd * J_ptr = task.getJacobianPtr();
        // const Eigen::MatrixXd * J_inv_ptr = task.getJacobianInversePtr();
        // Eigen::VectorXd null_space_projection = (identity - *J_ptr * *J_inv_ptr) * joint_velocities;
        tasks_[i].setDesiredPose(&desired_poses[i]);
        desired_joint_velocities += tasks_[i].solve();
    }

    return desired_joint_velocities;
}

Eigen::VectorXd IKSolver::integrateJointVelocities()
{
    // Integrate the joint velocities
    Eigen::VectorXd current_joint_positions = *current_joint_positions_ptr_;
    current_joint_positions += *desired_joint_velocities_ptr_ * dt_;
    return current_joint_positions;
}

uint8_t IKSolver::getNumberOfTasks()
{
    // Get the number of tasks
    return tasks_.size();
}

std::vector<uint8_t> IKSolver::getTasksM()
{
    // Get the dimension of each task
    std::vector<uint8_t> tasks_m;
    for (auto task : tasks_)
        tasks_m.push_back(task.getTaskM());
    return tasks_m;
}

void IKSolver::configureTasks()
{
    // TODO: Load the tasks from a YAML file.
    Task task = { 0, "motion", 6, 8 };
    task.setGainP(1.0);
    task.setDampingCoefficient(1e-2);
    tasks_.push_back(task);
}

void IKSolver::setCurrentJointPositionsPtr(Eigen::VectorXd* current_joint_positions_ptr)
{
    // Set the pointer to the current joint positions
    current_joint_positions_ptr_ = current_joint_positions_ptr;
}

void IKSolver::setDesiredJointPositionsPtr(Eigen::VectorXd* desired_joint_positions_ptr)
{
    // Set the pointer to the desired joint positions
    desired_joint_positions_ptr_ = desired_joint_positions_ptr;
}

void IKSolver::setDesiredJointVelocitiesPtr(Eigen::VectorXd* desired_joint_velocities_ptr)
{
    // Set the pointer to the desired joint velocities
    desired_joint_velocities_ptr_ = desired_joint_velocities_ptr;
}

// const Eigen::MatrixXd * IKSolver::getJacobianPtr(int task_id)
// {
//     // Get the Jacobian of a task
//     return tasks_[task_id].getJacobianPtr();
// }

// const Eigen::MatrixXd * IKSolver::getJacobianInversePtr(int task_id)
// {
//     // Get the inverse of Jacobian of a task
//     return tasks_[task_id].getJacobianInversePtr();
// }