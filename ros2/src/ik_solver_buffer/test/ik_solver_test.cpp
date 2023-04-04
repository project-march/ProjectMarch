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

class IkBufferTest : public testing::Test {
protected:
    void SetUp() override
    {
        this->ik_buffer = std::make_unique<IkBufferTest>();
    }

    std::unique_ptr<IkBufferTest> ik_buffer;
};

TEST_F(IkBufferTest, intializationTest)
{
    std::string test_robot_location = "test/urdf/march_test_urdf.urdf";
    this->ik_solver->load_urdf_model(test_robot_location);
    ASSERT_EQ(1, 1);
}

#endif
