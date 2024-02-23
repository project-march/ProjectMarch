//
// Created by AlexanderJamesBecoy on 2023-01-10.
//

#ifndef __clang_analyzer__
// NOLINTBEGIN
// Copyright 2023 Project March.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include <march_ik_solver/ik_solver.hpp>

class IkSolverTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        m_ik_solver = std::make_unique<IKSolver>();
        m_ik_solver->setDt(m_dt);
        m_ik_solver->setJointConfigurations(m_joint_names, 
            m_joint_position_lower_limits, m_joint_position_upper_limits,
            m_joint_velocity_lower_limits, m_joint_velocity_upper_limits);
    }

    void TearDown() override
    {
        m_ik_solver.reset();
    }

    void configureMotionTask()
    {
        std::vector<std::string> task_names = { "motion" };
        m_ik_solver->setTaskNames(task_names);
        m_ik_solver->createTask("motion", m_motion_task_reference_frame, m_motion_task_node_names,
            m_motion_task_workspace_dim, m_motion_task_configurations_dim, m_motion_task_gain_p, m_motion_task_gain_d,
            m_motion_task_gain_i, m_motion_task_damping_coefficient);
    }

    void configureStabilityTask()
    {
        std::vector<std::string> task_names = { "stability" };
        m_ik_solver->setTaskNames(task_names);
        m_ik_solver->createTask("stability", m_stability_task_reference_frame, m_stability_task_node_names,
            m_stability_task_workspace_dim, m_stability_task_configurations_dim, m_stability_task_gain_p,
            m_stability_task_gain_d, m_stability_task_gain_i, m_stability_task_damping_coefficient);
    }

    void configureAllTasks()
    {
        m_ik_solver->setTaskNames(m_task_names);
        for (unsigned int i = 0; i < m_task_names.size(); i++) {
            if (m_task_names[i] == "stability") {
                m_ik_solver->createTask(m_task_names[i], m_stability_task_reference_frame, m_stability_task_node_names,
                    m_stability_task_workspace_dim, m_stability_task_configurations_dim, m_stability_task_gain_p,
                    m_stability_task_gain_d, m_stability_task_gain_i, m_stability_task_damping_coefficient);
            } else if (m_task_names[i] == "motion") {
                m_ik_solver->createTask(m_task_names[i], m_motion_task_reference_frame, m_motion_task_node_names,
                    m_motion_task_workspace_dim, m_motion_task_configurations_dim, m_motion_task_gain_p,
                    m_motion_task_gain_d, m_motion_task_gain_i, m_motion_task_damping_coefficient);
            }
        }
    }

    void updateZeroDesiredTasks()
    {
        std::unordered_map<std::string, Eigen::VectorXd> desired_tasks;
        for (unsigned int i = 0; i < m_task_names.size(); i++) {
            desired_tasks[m_task_names[i]] = Eigen::VectorXd::Zero(m_task_configurations_dims[i]);
        }
        m_ik_solver->updateDesiredTasks(desired_tasks);
    }

    void updateZeroCurrentJointState()
    {
        std::vector<double> current_joint_positions, current_joint_velocities;
        for (unsigned int i = 0; i < m_joint_names.size(); i++) {
            current_joint_positions.push_back(0.0);
            current_joint_velocities.push_back(0.0);
        }
        m_ik_solver->updateCurrentJointState(current_joint_positions, current_joint_velocities);
    }

    // IKSolverNode::configureIKSolverParameters()
    double m_dt = 1e-3;
    std::vector<std::string> m_joint_names = { "left_hip_aa", "left_hip_fe", "left_knee", "left_ankle", "right_hip_aa",
        "right_hip_fe", "right_knee", "right_ankle" };
    std::vector<double> m_joint_position_lower_limits = { 10.0, 112.5, 125.0, 10.0, 10.0, 112.5, 125.0, 10.0 };
    std::vector<double> m_joint_position_upper_limits = { -15.0, -10.0, 0.0, -25.0, -15.0, -10.0, 0.0, -25.0 };
    std::vector<double> m_joint_velocity_lower_limits = { -100.0, -115.6, -243.6, -130.1, -100.0, -115.6, -243.6, -130.1 };
    std::vector<double> m_joint_velocity_upper_limits = { 100.0,  67.16,  208.9,  100.3,  100.0,  67.16,  208.9,  100.3 };

    // IKSolverNode::configureTasksParameters()
    std::vector<std::string> m_task_names = { "stability", "motion" };

    // IKSolverNode::configureTasksParameters() : Motion Task
    std::string m_motion_task_reference_frame = "body";
    std::vector<std::string> m_motion_task_node_names = { "left_ankle", "right_ankle" };
    unsigned int m_motion_task_workspace_dim = 12;
    unsigned int m_motion_task_configurations_dim = 8;
    std::vector<double> m_motion_task_gain_p = { 50.0, 100.0, 100.0, 0.0, 0.0, 0.0, 50.0, 100.0, 100.0, 0.0, 0.0, 0.0 };
    std::vector<double> m_motion_task_gain_d = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    std::vector<double> m_motion_task_gain_i = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double m_motion_task_damping_coefficient = 1e-6;

    // IKSolverNode::configureTasksParameters() : Stability Task
    std::string m_stability_task_reference_frame = "inertial";
    std::vector<std::string> m_stability_task_node_names = { "com" };
    unsigned int m_stability_task_workspace_dim = 2;
    unsigned int m_stability_task_configurations_dim = 8;
    std::vector<double> m_stability_task_gain_p = { 50.0, 5.0 };
    std::vector<double> m_stability_task_gain_d = { 0.0, 0.0 };
    std::vector<double> m_stability_task_gain_i = { 0.0, 0.0 };
    double m_stability_task_damping_coefficient = 1e-6;

    // Utility member variables
    std::vector<unsigned int> m_task_workspace_dims = { m_motion_task_workspace_dim };
    std::vector<unsigned int> m_task_configurations_dims = { m_motion_task_configurations_dim };

    std::unique_ptr<IKSolver> m_ik_solver;
    double m_error_tolerance = 1e-3;
};

TEST_F(IkSolverTest, test_should_be_able_to_configure_ik_solver_and_motion_task)
{
    ASSERT_NO_FATAL_FAILURE(configureMotionTask());
}

TEST_F(IkSolverTest, test_should_be_able_to_configure_ik_solver_and_stability_task)
{
    ASSERT_NO_FATAL_FAILURE(configureStabilityTask());
}

TEST_F(IkSolverTest, test_should_be_able_to_update_current_joint_state)
{
    configureAllTasks();
    ASSERT_NO_FATAL_FAILURE(updateZeroCurrentJointState());
}

TEST_F(IkSolverTest, test_should_be_able_to_update_desired_motion_task)
{
    configureMotionTask();
    std::unordered_map<std::string, Eigen::VectorXd> desired_tasks;
    desired_tasks.insert(std::make_pair("motion", Eigen::VectorXd::Zero(m_motion_task_workspace_dim)));
    ASSERT_NO_FATAL_FAILURE(m_ik_solver->updateDesiredTasks(desired_tasks));
}

TEST_F(IkSolverTest, test_should_be_able_to_update_desired_stability_task)
{
    configureStabilityTask();
    std::unordered_map<std::string, Eigen::VectorXd> desired_tasks;
    desired_tasks.insert(std::make_pair("stability", Eigen::VectorXd::Zero(m_stability_task_workspace_dim)));
    ASSERT_NO_FATAL_FAILURE(m_ik_solver->updateDesiredTasks(desired_tasks));
}

TEST_F(IkSolverTest, test_should_be_able_to_update_desired_all_tasks)
{
    configureAllTasks();
    std::unordered_map<std::string, Eigen::VectorXd> desired_tasks;
    desired_tasks.insert(std::make_pair("motion", Eigen::VectorXd::Zero(m_motion_task_workspace_dim)));
    desired_tasks.insert(std::make_pair("stability", Eigen::VectorXd::Zero(m_stability_task_workspace_dim)));
    ASSERT_NO_FATAL_FAILURE(m_ik_solver->updateDesiredTasks(desired_tasks));
}

TEST_F(IkSolverTest, test_should_be_able_to_solve_inverse_kinematics_in_motion_task)
{
    configureMotionTask();
    std::unordered_map<std::string, Eigen::VectorXd> desired_tasks;
    desired_tasks.insert(std::make_pair("motion", Eigen::VectorXd::Zero(m_motion_task_workspace_dim)));
    m_ik_solver->updateDesiredTasks(desired_tasks);
    updateZeroCurrentJointState();
    ASSERT_NO_FATAL_FAILURE(m_ik_solver->solveInverseKinematics());
}

// NOLINTEND
#endif // __clang_analyzer__