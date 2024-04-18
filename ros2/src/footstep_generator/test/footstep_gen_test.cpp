#ifndef __clang_analyzer__
// NOLINTBEGIN
// Copyright 2022 Project March.
#include "footstep_generator/footstep_generator.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <cmath>
#include <memory>
#include <utility>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
using testing::_;
using testing::ByMove;
using testing::Eq;
using testing::Return;

class FootstepGeneratorTest : public testing::Test {
protected:
    void SetUp() override
    {
        this->footstep_gen = std::make_unique<FootstepGenerator>();
    }

    std::unique_ptr<FootstepGenerator> footstep_gen;
};

TEST_F(FootstepGeneratorTest, gettersTest)
{
    ASSERT_EQ(this->footstep_gen->get_steps(), 20);
    ASSERT_EQ(this->footstep_gen->get_velocity_x(), 0.1);
    ASSERT_EQ(this->footstep_gen->get_velocity_y(), 0.0);
    ASSERT_EQ(this->footstep_gen->get_feet_spread(), 0.33);
}

TEST_F(FootstepGeneratorTest, getWalkedDistanceTest)
{
    double expected_dist_x = this->footstep_gen->get_steps() * this->footstep_gen->get_velocity_x();
    double expected_dist_y = this->footstep_gen->get_steps() * this->footstep_gen->get_velocity_y();
    const double margin = 0.5;

    geometry_msgs::msg::PoseArray footsteps = this->footstep_gen->generate_foot_placements(1, 2);
    double footstep_x = footsteps.poses[this->footstep_gen->get_steps() - 1].position.x;
    double footstep_y = footsteps.poses[this->footstep_gen->get_steps() - 1].position.y;

    ASSERT_LT(std::abs(footstep_x - expected_dist_x), margin);
    ASSERT_LT(std::abs(footstep_y - expected_dist_y), margin);
}

TEST_F(FootstepGeneratorTest, getAMountOfStepsTest)
{
    geometry_msgs::msg::PoseArray footsteps = this->footstep_gen->generate_foot_placements(1, 2);
    ASSERT_LT(sizeof(footsteps.poses) / sizeof(geometry_msgs::msg::Pose),
        static_cast<double>(this->footstep_gen->get_steps()));
}

TEST_F(FootstepGeneratorTest, getStepAndCloseDistance)
{
    double expected_dist_x = 1 * this->footstep_gen->get_velocity_x(); // One step distance
    double expected_dist_y = 1 * this->footstep_gen->get_velocity_y();
    const double margin = 0.5;

    geometry_msgs::msg::PoseArray footsteps = this->footstep_gen->generate_foot_placements(1, 3);
    double footstep_x = footsteps.poses[this->footstep_gen->get_steps() - 1].position.x;
    double footstep_y = footsteps.poses[this->footstep_gen->get_steps() - 1].position.y;

    ASSERT_LT(std::abs(footstep_x - expected_dist_x), margin);
    ASSERT_LT(std::abs(footstep_y - expected_dist_y), margin);
}
// NOLINTEND
#endif
