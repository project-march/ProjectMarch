#ifndef __clang_analyzer__
// NOLINTBEGIN
// Copyright 2022 Project March.
// #include "ik_solver.cpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "ik_solver/ik_solver.hpp"
#include <cmath>
#include <memory>
#include <utility>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
using testing::_;
using testing::ByMove;
using testing::Eq;
using testing::Return;

class IkSolverTest : public testing::Test {
protected:
    void SetUp() override
    {
        this->ik_solver = std::make_unique<IkSolver>();
    }

    std::unique_ptr<IkSolver> ik_solver;
};

TEST_F(IkSolverTest, intializationTest)
{
    std::string test_robot_location = "test/urdf/march_test_urdf.urdf";
    this->ik_solver->load_urdf_model(test_robot_location);
    ASSERT_EQ(1, 1);
}

TEST_F(IkSolverTest, setJacobianTest)
{
    // We just need to check if the jacobian is constructed correctly,
    // As pinnochio handles its ownunittests for constructing the Jacobian.
    std::string test_robot_location = "test/urdf/march_test_urdf.urdf";
    this->ik_solver->load_urdf_model(test_robot_location);
    ASSERT_EQ(0, this->ik_solver->set_jacobian());
}

TEST_F(IkSolverTest, jointGetterTest)
{
    // WE CAN TEST THIS BETTER LATER

    ASSERT_NE(-1, this->ik_solver->get_model_joints());
}

// ADD MORE FUNCTIONAL TESTS LATER

// TEST_F(IkSolverTest, failingTest)
// {
//     ASSERT_EQ(8, 80);
// }
// NOLINTEND
#endif
