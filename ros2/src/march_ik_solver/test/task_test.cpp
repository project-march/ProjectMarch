//
// Created by AlexanderJamesBecoy on 2023-01-10.
//

#ifndef __clang_analyzer__
// NOLINTBEGIN
// Copyright 2023 Project March.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <march_ik_solver/task.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#define WORKSPACE_DIMENSION 3
#define JOINT_DIMENSION 2

class TaskTest : public ::testing::Test {
public:
    TaskTest() = default;
    ~TaskTest() override = default;

protected:
    void SetUp() override
    {
        m_task = std::make_unique<Task>(m_task_name, m_reference_frame, m_task_m, m_task_n, m_dt);
        configureTaskParameters();
        configurePointerReferences();
        m_task->setUnitTest(true);
    }

    void TearDown() override
    {
        m_task.reset();
    }

    void configurePointerReferences()
    {
        m_task->setJointNamesPtr(&m_joint_names);
        m_task->setCurrentJointPositionsPtr(&m_current_joint_positions);
        m_task->setDesiredTask(m_desired_task);
    }

    void configureTaskParameters()
    {
        m_task->setCurrentTask(m_current_task);
        m_task->setNodeNames(m_node_names);
        m_task->setGainP(m_gain_p);
        m_task->setGainD(m_gain_d);
        m_task->setGainI(m_gain_i);
        m_task->setDampingCoefficient(m_damping_coefficient);
    }

    Eigen::MatrixXd setCalculateAndGetJacobianInverse(Eigen::MatrixXd jacobian)
    {
        m_task->setJacobian(jacobian);
        m_task->calculateJacobianInverse();
        return m_task->getJacobianInverse();
    }

    std::string m_task_name = "test";
    std::string m_reference_frame = "body";
    unsigned int m_task_m = WORKSPACE_DIMENSION;
    unsigned int m_task_n = JOINT_DIMENSION;
    std::vector<std::string> m_node_names = { "node_1", "node_2" };
    std::unique_ptr<Task> m_task;

    std::vector<std::string> m_joint_names = { "joint_1", "joint_2" };
    Eigen::VectorXd m_current_task = Eigen::VectorXd::Zero(WORKSPACE_DIMENSION);
    Eigen::VectorXd m_current_joint_positions = Eigen::VectorXd::Zero(JOINT_DIMENSION);
    Eigen::VectorXd m_desired_task = Eigen::VectorXd::Ones(WORKSPACE_DIMENSION);

    float m_dt = 0.001;
    std::vector<double> m_gain_p = { 1.0, 1.0, 1.0 };
    std::vector<double> m_gain_d = { 0.0, 0.0, 0.0 };
    std::vector<double> m_gain_i = { 0.0, 0.0, 0.0 };
    float m_damping_coefficient = 0.1;

    float m_test_jacobian_error_tolerance = 1e-6;
    float m_test_velocity_error_tolerance = 1e-6;
};

TEST_F(TaskTest, test_should_get_equal_constructor_parameter_name)
{
    ASSERT_EQ(m_task->getTaskName(), m_task_name);
}

TEST_F(TaskTest, test_should_get_equal_constructor_parameter_m)
{
    ASSERT_EQ(m_task->getTaskM(), m_task_m);
}

TEST_F(TaskTest, test_should_get_equal_constructor_parameter_n)
{
    ASSERT_EQ(m_task->getTaskN(), m_task_n);
}

TEST_F(TaskTest, test_should_get_equal_constructor_parameter_node_names)
{
    ASSERT_EQ(m_task->getNodeNames(), m_node_names);
}

TEST_F(TaskTest, test_should_set_and_get_reference_to_current_joint_names)
{
    ASSERT_EQ(m_task->getJointNamesPtr(), &m_joint_names);
}

TEST_F(TaskTest, test_should_set_and_get_reference_to_current_joint_positions)
{
    ASSERT_EQ(m_task->getCurrentJointPositionsPtr(), &m_current_joint_positions);
}

TEST_F(TaskTest, test_should_set_and_get_reference_to_desired_tasks)
{
    ASSERT_EQ(m_task->getDesiredTask(), m_desired_task);
}

TEST_F(TaskTest, test_should_calculate_error_given_current_task_equals_zeros_and_desired_task_equals_ones)
{
    Eigen::VectorXd expected_error = Eigen::VectorXd::Ones(WORKSPACE_DIMENSION);
    double expected_error_norm = sqrt(WORKSPACE_DIMENSION);

    Eigen::VectorXd actual_error = m_task->calculateError();
    double actual_error_norm = m_task->getErrorNorm();

    ASSERT_EQ(expected_error.rows(), actual_error.rows());
    ASSERT_EQ(expected_error_norm, actual_error_norm);
    ASSERT_EQ(expected_error, actual_error);
}

TEST_F(TaskTest, test_should_get_correct_jacobian_inverse_using_complete_orthogonal_decomposition)
{
    Eigen::MatrixXd expected_jacobian_inverse = Eigen::MatrixXd(JOINT_DIMENSION, WORKSPACE_DIMENSION);
    expected_jacobian_inverse << 5.16381, -4.83619, 0.156006, -4.83619, 5.16381, 0.156006;
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Ones(WORKSPACE_DIMENSION, JOINT_DIMENSION);

    Eigen::MatrixXd actual_jacobian_inverse = setCalculateAndGetJacobianInverse(jacobian);

    ASSERT_EQ(actual_jacobian_inverse.rows(), expected_jacobian_inverse.rows());
    ASSERT_EQ(actual_jacobian_inverse.cols(), expected_jacobian_inverse.cols());
    ASSERT_TRUE(actual_jacobian_inverse.isApprox(expected_jacobian_inverse, m_test_jacobian_error_tolerance));
}

TEST_F(TaskTest, test_should_get_correct_joint_velocity_after_solving_for_dummy_desired_task_and_zero_current_task)
{
    Eigen::VectorXd expected_joint_velocities = 0.48361934 * Eigen::VectorXd::Ones(JOINT_DIMENSION);
    Eigen::VectorXd desired_task = Eigen::VectorXd::Ones(WORKSPACE_DIMENSION);
    Eigen::VectorXd current_task = Eigen::VectorXd::Zero(WORKSPACE_DIMENSION);
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Ones(WORKSPACE_DIMENSION, JOINT_DIMENSION);

    m_task->setDesiredTask(desired_task);
    m_task->setCurrentTask(current_task);
    m_task->setJacobian(jacobian);
    m_task->calculateJacobianInverse();
    Eigen::VectorXd actual_joint_velocities = m_task->solveTask();

    ASSERT_EQ(actual_joint_velocities.rows(), expected_joint_velocities.rows());
    ASSERT_TRUE(actual_joint_velocities.isApprox(expected_joint_velocities, m_test_velocity_error_tolerance));
}

TEST_F(TaskTest, test_should_be_able_to_get_relevant_errors_in_extended_task)
{
    Eigen::VectorXd expected_error = Eigen::VectorXd(6);
    expected_error << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;
    double expected_error_norm = 1.7320508075688772935274463415058723669428052538103806280558069794;

    m_task->setTaskM(6);
    m_task->setTaskN(2);
    std::vector<double> gain_p = { 1.0, 1.0, 1.0, 0.0, 0.0, 0.0 };
    std::vector<double> gain_d = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    std::vector<double> gain_i = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    m_task->setGainP(gain_p);
    m_task->setGainD(gain_d);
    m_task->setGainI(gain_i);

    m_task->setDesiredTask(Eigen::VectorXd::Ones(6));
    m_task->setCurrentTask(Eigen::VectorXd::Zero(6));
    Eigen::VectorXd actual_error = m_task->calculateError();
    double actual_error_norm = m_task->getErrorNorm();

    std::cout << "actual_error:\n" << actual_error << std::endl;
    std::cout << "actual_error_norm:\n" << actual_error_norm << std::endl;
    ASSERT_EQ(actual_error.rows(), expected_error.rows());
    ASSERT_EQ(actual_error_norm, expected_error_norm);
    ASSERT_TRUE(actual_error.isApprox(expected_error));
}

// NOLINTEND
#endif // __clang_analyzer__