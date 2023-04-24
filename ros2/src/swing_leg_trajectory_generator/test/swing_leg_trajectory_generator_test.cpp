//
// Created by march on 13-3-23.
//
#ifndef __clang_analyzer__
// NOLINTBEGIN

#include "swing_leg_trajectory_generator.hpp"
#include <gtest/gtest.h>
#include <memory>

class SwingLegTrajectoryGeneratorTest : public testing::Test {
protected:
    std::unique_ptr<SwingLegTrajectoryGenerator> swing_leg_generator;

private:
    void SetUp() override
    {
        swing_leg_generator = std::make_unique<SwingLegTrajectoryGenerator>();
    }
};

TEST_F(SwingLegTrajectoryGeneratorTest, getCurveTest)
{
    auto expected = BezierCurve();
    auto start_point = Point();
    start_point.x = 0;
    start_point.y = 0;
    start_point.z = 0;

    auto left_point = Point();
    left_point.x = 25;
    left_point.y = 50;
    left_point.z = 0;

    auto right_point = Point();
    right_point.x = 75;
    right_point.y = 50;
    right_point.z = 0;

    auto end_point = Point();
    end_point.x = 100;
    end_point.y = 0;
    end_point.z = 0;

    expected.points.push_back(start_point);
    expected.points.push_back(left_point);
    expected.points.push_back(right_point);
    expected.points.push_back(end_point);
    auto actual = swing_leg_generator->getCurve();
    ASSERT_EQ(expected, actual);
}

TEST_F(SwingLegTrajectoryGeneratorTest, getPointTest)
{
    auto expected = geometry_msgs::msg::Point();
    expected.x = 50.0;
    expected.y = 37.5;
    expected.z = 0;
    double t = 0.5;
    auto actual = swing_leg_generator->getPoint(swing_leg_generator->getCurve().points, t);
    ASSERT_EQ(expected.x, actual.x);
    ASSERT_EQ(expected.y, actual.y);
    ASSERT_EQ(expected.z, actual.z);
}

TEST_F(SwingLegTrajectoryGeneratorTest, setPointsTest)
{
    auto expected = std::vector<Point>();
    auto start_point = Point();
    start_point.x = 1;
    start_point.y = 1;
    start_point.z = 1;

    auto left_point = Point();
    left_point.x = 10;
    left_point.y = 10;
    left_point.z = 10;

    auto right_point = Point();
    right_point.x = 20;
    right_point.y = 20;
    right_point.z = 20;

    auto end_point = Point();
    end_point.x = 30;
    end_point.y = 30;
    end_point.z = 30;

    expected.push_back(start_point);
    expected.push_back(left_point);
    expected.push_back(right_point);
    expected.push_back(end_point);
    swing_leg_generator->setPoints(expected);
    ASSERT_EQ(expected, swing_leg_generator->getCurve().points);
}

// NOLINTEND
#endif