//
// Created by AlexanderJamesBecoy on 2023-01-10.
//

#ifndef __clang_analyzer__
// NOLINTBEGIN
// Copyright 2023 Project March.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include <march_ik_solver/task.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#define WORKSPACE_DIMENSION 3
#define JOINT_DIMENSION 2

class TaskTest : public ::testing::Test
{
public:
    TaskTest() = default;
    ~TaskTest() override = default;

protected:
    void SetUp() override
    {
        m_task = std::make_unique<Task>(m_task_id, m_task_name, m_task_m, m_task_n, m_node_names);
        configurePointerReferences();
        configureTask();
        m_task->setUnitTest(true);
    }

    void TearDown() override
    {
        m_task.reset();
    }

    void configurePointerReferences()
    {
        m_task->setCurrentJointNamesPtr(&m_current_joint_names);
        m_task->setCurrentJointPositionsPtr(&m_current_joint_positions);
        m_task->setDesiredPosesPtr(&m_desired_poses);
    }

    void configureTask()
    {
        m_task->setCurrentPose(m_current_pose);
        m_task->setGainP(m_gain_p);
        m_task->setGainD(m_gain_d);
        m_task->setGainI(m_gain_i);
        m_task->setDt(m_dt);
        m_task->setDampingCoefficient(m_damping_coefficient);
    }

    Eigen::MatrixXd setCalculateAndGetJacobianInverse(Eigen::MatrixXd jacobian)
    {
        m_task->setJacobian(jacobian);
        m_task->calculateJacobianInverse();
        return *m_task->getJacobianInversePtr();
    }

    unsigned int m_task_id = 0;
    std::string m_task_name = "test";
    unsigned int m_task_m = WORKSPACE_DIMENSION;
    unsigned int m_task_n = JOINT_DIMENSION;
    std::vector<std::string> m_node_names = {"node_1", "node_2"};
    std::unique_ptr<Task> m_task;

    std::vector<std::string> m_current_joint_names = {"joint_1", "joint_2"};
    Eigen::VectorXd m_current_pose = Eigen::VectorXd::Zero(WORKSPACE_DIMENSION);
    Eigen::VectorXd m_current_joint_positions = Eigen::VectorXd::Zero(JOINT_DIMENSION);
    std::vector<Eigen::VectorXd> m_desired_poses = {Eigen::VectorXd::Ones(WORKSPACE_DIMENSION), Eigen::VectorXd::Ones(WORKSPACE_DIMENSION)};

    float m_dt = 0.001;
    float m_gain_p = 1.0;
    float m_gain_d = 0.0;
    float m_gain_i = 0.0;
    float m_damping_coefficient = 0.1;

    float m_test_error_tolerance = 1e-6;
};

TEST_F(TaskTest, test_should_get_equal_constructor_parameter_values)
{
    ASSERT_EQ(m_task->getTaskID(), m_task_id);
    ASSERT_EQ(m_task->getTaskName(), m_task_name);
    ASSERT_EQ(m_task->getTaskM(), m_task_m);
    ASSERT_EQ(m_task->getTaskN(), m_task_n);
    ASSERT_EQ(m_task->getNodeNames(), m_node_names);
}

TEST_F(TaskTest, test_should_set_and_get_reference_to_current_joint_names)
{
    ASSERT_EQ(m_task->getCurrentJointNamesPtr(), &m_current_joint_names);
}

TEST_F(TaskTest, test_should_set_and_get_reference_to_current_joint_positions)
{
    ASSERT_EQ(m_task->getCurrentJointPositionsPtr(), &m_current_joint_positions);
}

TEST_F(TaskTest, test_should_set_and_get_reference_to_desired_poses)
{
    ASSERT_EQ(m_task->getDesiredPosesPtr(), &m_desired_poses);
}

TEST_F(TaskTest, test_should_calculate_error_given_current_pose_equals_zeros_and_desired_pose_equals_ones)
{
    Eigen::VectorXd expected_error = Eigen::VectorXd::Ones(WORKSPACE_DIMENSION);
    double expected_error_norm = sqrt(WORKSPACE_DIMENSION);

    Eigen::VectorXd actual_error = m_task->calculateError();
    double actual_error_norm = m_task->getErrorNorm();

    ASSERT_EQ(expected_error.rows(), actual_error.rows());
    ASSERT_EQ(expected_error_norm, actual_error_norm);
    ASSERT_EQ(expected_error, actual_error);
}

TEST_F(TaskTest, test_should_get_correct_jacobian_inverse_given_jacobian_of_ones_of_underactuated_robot)
{
    Eigen::MatrixXd expected_jacobian_inverse = 0.16393443 * Eigen::MatrixXd::Ones(JOINT_DIMENSION, WORKSPACE_DIMENSION);
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Ones(WORKSPACE_DIMENSION, JOINT_DIMENSION);

    Eigen::MatrixXd actual_jacobian_inverse = setCalculateAndGetJacobianInverse(jacobian);

    ASSERT_EQ(actual_jacobian_inverse.rows(), expected_jacobian_inverse.rows());
    ASSERT_EQ(actual_jacobian_inverse.cols(), expected_jacobian_inverse.cols());
    ASSERT_TRUE(actual_jacobian_inverse.isApprox(expected_jacobian_inverse, m_test_error_tolerance));
}

TEST_F(TaskTest, test_should_get_correct_jacobian_inverse_given_jacobian_of_ones_of_overactuated_robot)
{
    m_task->setTaskM(JOINT_DIMENSION);
    m_task->setTaskN(WORKSPACE_DIMENSION);
    Eigen::MatrixXd expected_jacobian_inverse = 0.16393443 * Eigen::MatrixXd::Ones(WORKSPACE_DIMENSION, JOINT_DIMENSION);
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Ones(JOINT_DIMENSION, WORKSPACE_DIMENSION);

    Eigen::MatrixXd actual_jacobian_inverse = setCalculateAndGetJacobianInverse(jacobian);

    ASSERT_EQ(actual_jacobian_inverse.rows(), expected_jacobian_inverse.rows());
    ASSERT_EQ(actual_jacobian_inverse.cols(), expected_jacobian_inverse.cols());
    ASSERT_TRUE(actual_jacobian_inverse.isApprox(expected_jacobian_inverse, m_test_error_tolerance));
}

TEST_F(TaskTest, test_should_get_correct_jacobian_inverse_given_jacobian_of_ones_of_square_robot)
{
    m_task->setTaskN(WORKSPACE_DIMENSION);
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Ones(WORKSPACE_DIMENSION, WORKSPACE_DIMENSION);
    Eigen::MatrixXd expected_jacobian_inverse;
    expected_jacobian_inverse.resize(WORKSPACE_DIMENSION, WORKSPACE_DIMENSION);
    expected_jacobian_inverse << 6.77419355, -3.22580645, -3.22580645,
                                -3.22580645, 6.77419355, -3.22580645,
                                -3.22580645, -3.22580645, 6.77419355;
                                
    Eigen::MatrixXd actual_jacobian_inverse = setCalculateAndGetJacobianInverse(jacobian);

    ASSERT_EQ(actual_jacobian_inverse.rows(), expected_jacobian_inverse.rows());
    ASSERT_EQ(actual_jacobian_inverse.cols(), expected_jacobian_inverse.cols());
    ASSERT_TRUE(actual_jacobian_inverse.isApprox(expected_jacobian_inverse, m_test_error_tolerance));
}

TEST_F(TaskTest, test_should_get_correct_joint_velocity_after_solving_for_dummy_desired_pose_and_zero_current_pose)
{
    Eigen::VectorXd expected_joint_velocities = 0.49180328 * Eigen::VectorXd::Ones(JOINT_DIMENSION);
    std::vector<Eigen::VectorXd> desired_poses = {Eigen::VectorXd::Ones(WORKSPACE_DIMENSION)};
    Eigen::VectorXd current_pose = Eigen::VectorXd::Zero(WORKSPACE_DIMENSION);
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Ones(WORKSPACE_DIMENSION, JOINT_DIMENSION);

    m_task->setDesiredPosesPtr(&desired_poses);
    m_task->setCurrentPose(current_pose);
    m_task->setJacobian(jacobian);
    Eigen::VectorXd actual_joint_velocities = m_task->solve();

    ASSERT_EQ(actual_joint_velocities.rows(), expected_joint_velocities.rows());
    ASSERT_TRUE(actual_joint_velocities.isApprox(expected_joint_velocities, m_test_error_tolerance));
}

// NOLINTEND
#endif  // __clang_analyzer__