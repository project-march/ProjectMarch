#include "march_ik_solver/ik_solver.hpp"

#include <string>
#include <algorithm>
#include <math.h>
#include <boost/algorithm/clamp.hpp>

Eigen::VectorXd IKSolver::solve()
{
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
        desired_joint_velocities.noalias() += m_tasks[i].solve();
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "IKSolver::solve(): Solved task %s", m_tasks[i].getTaskName().c_str());
    }

    return desired_joint_velocities;
}

Eigen::VectorXd IKSolver::integrateJointVelocities()
{
    double dt = 1e-2; // TODO: Get dt from the integral_dt_ptr.
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "IKSolver::integrateJointVelocities(): Integrating joint velocities with dt: %f", dt);

    // Eigen::VectorXd desired_joint_velocities = *m_desired_joint_velocities_ptr;
    // RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "IKSolver::integrateJointVelocities(): Desired joint velocities: %f, %f, %f, %f, %f, %f, %f, %f, %f",
    //     desired_joint_velocities(0), desired_joint_velocities(1), desired_joint_velocities(2), desired_joint_velocities(3), 
    //     desired_joint_velocities(4), desired_joint_velocities(5), desired_joint_velocities(6), desired_joint_velocities(7));

    // Eigen::VectorXd current_joint_positions = *m_current_joint_positions_ptr;
    // RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "IKSolver::integrateJointVelocities(): Current joint positions: %f, %f, %f, %f, %f, %f, %f, %f, %f",
    //     current_joint_positions(0), current_joint_positions(1), current_joint_positions(2), current_joint_positions(3), 
    //     current_joint_positions(4), current_joint_positions(5), current_joint_positions(6), current_joint_positions(7));

    Eigen::VectorXd desired_joint_positions;
    desired_joint_positions.noalias() = (*m_current_joint_positions_ptr) + (*m_desired_joint_velocities_ptr) * dt;
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "IKSolver::integrateJointVelocities(): Desired joint positions: %f, %f, %f, %f, %f, %f, %f, %f, %f",
        desired_joint_positions(0), desired_joint_positions(1), desired_joint_positions(2), desired_joint_positions(3), 
        desired_joint_positions(4), desired_joint_positions(5), desired_joint_positions(6), desired_joint_positions(7));

    Eigen::VectorXd clamped_desired_joint_positions = clampJointLimits(desired_joint_positions);
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "IKSolver::integrateJointVelocities(): Limited desired joint positions: %f, %f, %f, %f, %f, %f, %f, %f, %f",
        clamped_desired_joint_positions(0), clamped_desired_joint_positions(1), clamped_desired_joint_positions(2), clamped_desired_joint_positions(3), 
        clamped_desired_joint_positions(4), clamped_desired_joint_positions(5), clamped_desired_joint_positions(6), clamped_desired_joint_positions(7));
    return clamped_desired_joint_positions;
}

std::vector<double> IKSolver::getTasksError()
{
    std::vector<double> tasks_error;
    for (long unsigned int i = 0; i < m_tasks.size(); i++)
    {
        tasks_error.push_back(m_tasks[i].getErrorNorm());
    }
    return tasks_error;
}

void IKSolver::setNJoints(int n_joints)
{
    m_n_joints = n_joints;
}

void IKSolver::setJointLimits(std::vector<double> lower_joint_limits, std::vector<double> upper_joint_limits)
{
    for (int i = 0; i < m_n_joints; i++)
    {
        double lower_joint_limit_rad = lower_joint_limits[i] * M_PI / 180.0;
        double upper_joint_limit_rad = upper_joint_limits[i] * M_PI / 180.0;
        m_joint_limits.push_back({lower_joint_limit_rad, upper_joint_limit_rad});
    }
}

void IKSolver::setTasks(std::vector<Task> tasks)
{
    m_tasks = tasks;
}

void IKSolver::configureTasks(std::vector<Eigen::VectorXd> * desired_poses_ptr)
{
    for (long unsigned int i = 0; i < m_tasks.size(); i++)
    {
        m_tasks[i].setCurrentJointPositionsPtr(m_current_joint_positions_ptr);
        m_tasks[i].setDesiredPosesPtr(desired_poses_ptr);
    }
}

void IKSolver::setIntegralDtPtr(uint32_t* integral_dt_ptr)
{
    m_integral_dt_ptr = integral_dt_ptr;
}

Eigen::VectorXd IKSolver::clampJointLimits(Eigen::VectorXd desired_joint_positions)
{
    Eigen::VectorXd limited_joint_positions = desired_joint_positions;
    for (int i = 0; i < m_n_joints; i++)
    {
        limited_joint_positions(i) = boost::algorithm::clamp(desired_joint_positions(i), m_joint_limits[i][0], m_joint_limits[i][1]);
    }
    return limited_joint_positions;
}

void IKSolver::setCurrentJointPositionsPtr(Eigen::VectorXd* current_joint_positions_ptr, std::vector<std::string> * joint_names_ptr)
{
    m_current_joint_positions_ptr = current_joint_positions_ptr;

    for (int i = 0; i < m_n_joints; i++)
    {
        m_tasks[i].setCurrentJointNamesPtr(joint_names_ptr);
    }

    for (auto & task : m_tasks)
    {
        task.setCurrentJointNamesPtr(joint_names_ptr);
    }
}

void IKSolver::setDesiredJointPositionsPtr(Eigen::VectorXd* desired_joint_positions_ptr)
{
    m_desired_joint_positions_ptr = desired_joint_positions_ptr;
}

void IKSolver::setDesiredJointVelocitiesPtr(Eigen::VectorXd* desired_joint_velocities_ptr)
{
    m_desired_joint_velocities_ptr = desired_joint_velocities_ptr;
}