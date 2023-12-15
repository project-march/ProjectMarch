#include "march_ik_solver/ik_solver.hpp"

#include <string>
#include <algorithm>
#include <math.h>
#include <boost/algorithm/clamp.hpp>

IKSolver::IKSolver()
{
    // Set the joint limits.
    // TODO: Load this from a YAML file instead of this hard-coded array.
    std::array<double,2> joint_limits_LHAA = { -15.0, 10.0 };
    std::array<double,2> joint_limits_LHFE = { -10.0, 112.5 };
    std::array<double,2> joint_limits_LKFE = {  0.0, 125.0 };
    std::array<double,2> joint_limits_LADPF = { -25.0, 10.0 };
    std::array<double,2> joint_limits_RHAA = { -15.0, 10.0 };
    std::array<double,2> joint_limits_RHFE = { -10.0, 112.5 };
    std::array<double,2> joint_limits_RKFE = {  0.0, 125.0 };
    std::array<double,2> joint_limits_RADPF = { -25.0, 10.0 };
    joint_limits_.push_back(joint_limits_LHAA);
    joint_limits_.push_back(joint_limits_LHFE);
    joint_limits_.push_back(joint_limits_LKFE);
    joint_limits_.push_back(joint_limits_LADPF);
    joint_limits_.push_back(joint_limits_RHAA);
    joint_limits_.push_back(joint_limits_RHFE);
    joint_limits_.push_back(joint_limits_RKFE);
    joint_limits_.push_back(joint_limits_RADPF);

    // Convert the joint limits from degrees to radians.
    for (long unsigned int i = 0; i < joint_limits_.size(); i++)
    {
        joint_limits_[i][0] = M_PI * joint_limits_[i][0] / 180.0;
        joint_limits_[i][1] = M_PI * joint_limits_[i][1] / 180.0;
    }

    for (long unsigned int i = 0; i < joint_limits_.size(); i++)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "IKSolver::IKSolver(): Joint %d limits: %f, %f", i, joint_limits_[i][0], joint_limits_[i][1]);
    }
}

Eigen::VectorXd IKSolver::solve()
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
        // tasks_[i].setDesiredPose(&desired_poses[i]);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "IKSolver::solve(): Solving task %s", tasks_[i].getTaskName().c_str());
        desired_joint_velocities += tasks_[i].solve();
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "IKSolver::solve(): Solved task %s", tasks_[i].getTaskName().c_str());
    }

    return desired_joint_velocities;
}

Eigen::VectorXd IKSolver::integrateJointVelocities()
{
    // Integrate the joint velocities
    double dt = 0.0005;
    Eigen::VectorXd desired_joint_positions = (*current_joint_positions_ptr_) + (*desired_joint_velocities_ptr_) * dt;
    // return desired_joint_positions;

    Eigen::VectorXd limited_desired_joint_positions = setJointLimits(desired_joint_positions);
    return limited_desired_joint_positions;
}

void IKSolver::setNJoints(int n_joints)
{
    // Set the number of joints
    n_joints_ = n_joints;
}

void IKSolver::setJointLimits(std::vector<std::array<double,2>> joint_limits)
{
    // Set the joint limits
    joint_limits_ = joint_limits;
}

void IKSolver::setTasks(std::vector<Task> tasks)
{
    // Set the tasks
    tasks_ = tasks;
}

void IKSolver::configureTasks(std::vector<Eigen::VectorXd> * desired_poses_ptr)
{
    // Set the current and desired pose.
    for (long unsigned int i = 0; i < tasks_.size(); i++)
    {
        tasks_[i].setCurrentJointPositionsPtr(current_joint_positions_ptr_);
        tasks_[i].setDesiredPosesPtr(desired_poses_ptr);
    }
}

void IKSolver::setIntegralDtPtr(uint32_t* integral_dt_ptr)
{
    // Set the pointer to the integral time step
    integral_dt_ptr_ = integral_dt_ptr;
}

Eigen::VectorXd IKSolver::setJointLimits(Eigen::VectorXd desired_joint_positions)
{
    // Set the joint limits
    // Eigen::VectorXd limited_joint_positions = Eigen::VectorXd::Zero(n_joints_);
    Eigen::VectorXd limited_joint_positions = desired_joint_positions;
    for (int i = 0; i < n_joints_; i++)
    {
        limited_joint_positions(i) = boost::algorithm::clamp(desired_joint_positions(i), joint_limits_[i][0], joint_limits_[i][1]);
    }
    return limited_joint_positions;
    // return desired_joint_positions;
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

std::vector<double> IKSolver::getPose(const Eigen::VectorXd * joint_positions)
{
    // Get the pose of the end-effector
    Eigen::VectorXd pose = tasks_[tasks_.size() - 1].getPose(joint_positions);
    return std::vector<double>(pose.data(), pose.data() + pose.size());
}