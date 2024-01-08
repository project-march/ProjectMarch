#include "march_ik_solver/ik_solver.hpp"

#include <string>
#include <algorithm>
#include <math.h>
#include <boost/algorithm/clamp.hpp>

Eigen::VectorXd IKSolver::solve()
{
    // Solve the IK problem
    Eigen::VectorXd desired_joint_velocities = Eigen::VectorXd::Zero(m_n_joints);
    // TODO: Uncomment code after MVP.
    // Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(m_n_joints, m_n_joints);

    // Calculate the joint velocities from the tasks
    for (long unsigned int i = 0; i < m_tasks.size(); i++) // TODO: Zip the tasks and desired poses.
    {
        // const Eigen::MatrixXd * J_ptr = task.getJacobianPtr();
        // const Eigen::MatrixXd * J_inv_ptr = task.getJacobianInversePtr();
        // Eigen::VectorXd null_space_projection = (identity - *J_ptr * *J_inv_ptr) * joint_velocities;
        // m_tasks[i].setDesiredPose(&desired_poses[i]);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "IKSolver::solve(): Solving task %s", m_tasks[i].getTaskName().c_str());
        desired_joint_velocities += m_tasks[i].solve();
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "IKSolver::solve(): Solved task %s", m_tasks[i].getTaskName().c_str());
    }

    return desired_joint_velocities;
}

Eigen::VectorXd IKSolver::integrateJointVelocities()
{
    // Integrate the joint velocities
    double dt = 0.05;
    Eigen::VectorXd desired_joint_positions = (*m_current_joint_positions_ptr) + (*m_desired_joint_velocities_ptr) * dt;
    // return desired_joint_positions;

    Eigen::VectorXd limited_desired_joint_positions = clampJointLimits(desired_joint_positions);
    return limited_desired_joint_positions;
}

std::vector<double> IKSolver::getTasksError()
{
    // Get the error of the tasks
    std::vector<double> tasks_error;
    for (long unsigned int i = 0; i < m_tasks.size(); i++)
    {
        tasks_error.push_back(m_tasks[i].getErrorNorm());
    }
    return tasks_error;
}

void IKSolver::setNJoints(int n_joints)
{
    // Set the number of joints
    m_n_joints = n_joints;
}

void IKSolver::setJointLimits(std::vector<double> lower_joint_limits, std::vector<double> upper_joint_limits)
{
    // Set the joint limits
    for (int i = 0; i < m_n_joints; i++)
    {
        m_joint_limits.push_back({lower_joint_limits[i], upper_joint_limits[i]});
    }
}

void IKSolver::setTasks(std::vector<Task> tasks)
{
    // Set the tasks
    m_tasks = tasks;
}

void IKSolver::configureTasks(std::vector<Eigen::VectorXd> * desired_poses_ptr)
{
    // Set the current and desired pose.
    for (long unsigned int i = 0; i < m_tasks.size(); i++)
    {
        m_tasks[i].setCurrentJointPositionsPtr(m_current_joint_positions_ptr);
        m_tasks[i].setDesiredPosesPtr(desired_poses_ptr);
    }
}

void IKSolver::setIntegralDtPtr(uint32_t* integral_dt_ptr)
{
    // Set the pointer to the integral time step
    m_integral_dt_ptr = integral_dt_ptr;
}

Eigen::VectorXd IKSolver::clampJointLimits(Eigen::VectorXd desired_joint_positions)
{
    // Set the joint limits
    // Eigen::VectorXd limited_joint_positions = Eigen::VectorXd::Zero(m_n_joints);
    Eigen::VectorXd limited_joint_positions = desired_joint_positions;
    for (int i = 0; i < m_n_joints; i++)
    {
        limited_joint_positions(i) = boost::algorithm::clamp(desired_joint_positions(i), m_joint_limits[i][0], m_joint_limits[i][1]);
    }
    return limited_joint_positions;
    // return desired_joint_positions;
}

void IKSolver::setCurrentJointPositionsPtr(Eigen::VectorXd* current_joint_positions_ptr, std::vector<std::string> * joint_names_ptr)
{
    // Set the pointer to the current joint positions
    m_current_joint_positions_ptr = current_joint_positions_ptr;

    // Set the current joint names
    for (int i = 0; i < m_n_joints; i++)
    {
        m_tasks[i].setCurrentJointNamesPtr(joint_names_ptr);
    }

    // Set the pointer to the joint names in each task
    for (auto & task : m_tasks)
    {
        task.setCurrentJointNamesPtr(joint_names_ptr);
    }
}

void IKSolver::setDesiredJointPositionsPtr(Eigen::VectorXd* desired_joint_positions_ptr)
{
    // Set the pointer to the desired joint positions
    m_desired_joint_positions_ptr = desired_joint_positions_ptr;
}

void IKSolver::setDesiredJointVelocitiesPtr(Eigen::VectorXd* desired_joint_velocities_ptr)
{
    // Set the pointer to the desired joint velocities
    m_desired_joint_velocities_ptr = desired_joint_velocities_ptr;
}

// const Eigen::MatrixXd * IKSolver::getJacobianPtr(int task_id)
// {
//     // Get the Jacobian of a task
//     return m_tasks[task_id].getJacobianPtr();
// }

// const Eigen::MatrixXd * IKSolver::getJacobianInversePtr(int task_id)
// {
//     // Get the inverse of Jacobian of a task
//     return m_tasks[task_id].getJacobianInversePtr();
// }

// std::vector<double> IKSolver::getPose(const Eigen::VectorXd * joint_positions)
// {
//     // Get the pose of the end-effector
//     Eigen::VectorXd pose = m_tasks[m_tasks.size() - 1].getPose(joint_positions);
//     return std::vector<double>(pose.data(), pose.data() + pose.size());
// }