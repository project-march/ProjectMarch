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

        auto points = std::vector<Point>();
        auto start_point = Point();
        start_point.x = 0.0;
        start_point.y = 0.0;
        start_point.z = 0.0;

        auto left_point = Point();
        left_point.x = 0.05;
        left_point.y = 0.0;
        left_point.z = 0.1;

        auto right_point = Point();
        right_point.x = 0.15;
        right_point.y = 0.0;
        right_point.z = 0.1;

        auto end_point = Point();
        end_point.x = 0.2;
        end_point.y = 0.0;
        end_point.z = 0.0;

        points.push_back(start_point);
        points.push_back(left_point);
        points.push_back(right_point);
        points.push_back(end_point);
        swing_leg_generator->set_points(points);
    }
};

TEST_F(SwingLegTrajectoryGeneratorTest, setGetStepLengthTest)
{
    double expected = 0.9;
    swing_leg_generator->set_step_length(expected);
    ASSERT_EQ(expected, swing_leg_generator->get_step_length());
}

TEST_F(SwingLegTrajectoryGeneratorTest, updatePointsTest)
{
    auto expected = std::vector<Point>();
    auto start_point = Point();
    start_point.x = 0;
    start_point.y = 0;
    start_point.z = 0;

    auto left_point = Point();
    left_point.x = 5;
    left_point.y = 0;
    left_point.z = 10;

    auto right_point = Point();
    right_point.x = 15;
    right_point.y = 0;
    right_point.z = 10;

    auto end_point = Point();
    end_point.x = 20;
    end_point.y = 0;
    end_point.z = 0;

    expected.push_back(start_point);
    expected.push_back(left_point);
    expected.push_back(right_point);
    expected.push_back(end_point);
    swing_leg_generator->update_points(swing_leg_generator->get_curve().points, 20);
    auto actual = swing_leg_generator->get_curve().points;

    ASSERT_EQ(expected.size(), actual.size());
    for (size_t i = 0; i < expected.size(); i++) {
        ASSERT_EQ(expected.at(i).x, actual.at(i).x);
        ASSERT_EQ(expected.at(i).y, actual.at(i).y);
        ASSERT_EQ(expected.at(i).z, actual.at(i).z);
    }
}

TEST_F(SwingLegTrajectoryGeneratorTest, getCurveTest)
{
    auto expected = BezierCurve();
    auto start_point = Point();
    start_point.x = 0;
    start_point.y = 0;
    start_point.z = 0;

    auto left_point = Point();
    left_point.x = 25;
    left_point.y = 0;
    left_point.z = 50;

    auto right_point = Point();
    right_point.x = 75;
    right_point.y = 0;
    right_point.z = 50;

    auto end_point = Point();
    end_point.x = 100;
    end_point.y = 0;
    end_point.z = 0;

    expected.points.push_back(start_point);
    expected.points.push_back(left_point);
    expected.points.push_back(right_point);
    expected.points.push_back(end_point);
    expected.point_amount = 75;

    auto actual = swing_leg_generator->get_curve();
    ASSERT_EQ(expected.points.size(), actual.points.size());
    for (size_t i = 0; i < expected.points.size(); i++) {
        ASSERT_EQ(expected.points.at(i).x / 500, actual.points.at(i).x);
        ASSERT_EQ(expected.points.at(i).y / 500, actual.points.at(i).y);
        ASSERT_EQ(expected.points.at(i).z / 500, actual.points.at(i).z);
    }
    ASSERT_EQ(expected.point_amount, actual.point_amount);
}

TEST_F(SwingLegTrajectoryGeneratorTest, getPointTest)
{
    auto expected = geometry_msgs::msg::Point();
    expected.x = 0.1;
    expected.y = 0;
    expected.z = 0.075;
    double t = 0.5;
    auto actual = swing_leg_generator->get_point(swing_leg_generator->get_curve().points, t);
    ASSERT_LT(std::abs(expected.x - actual.x), 1e-8);
    ASSERT_LT(std::abs(expected.y - actual.y), 1e-8);
    ASSERT_LT(std::abs(expected.z - actual.z), 1e-8);
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
    end_point.x = 100;
    end_point.y = 30;
    end_point.z = 30;

    expected.push_back(start_point);
    expected.push_back(left_point);
    expected.push_back(right_point);
    expected.push_back(end_point);
    swing_leg_generator->set_points(expected);

    auto actual = swing_leg_generator->get_curve().points;
    ASSERT_EQ(expected.size(), actual.size());
    for (size_t i = 0; i < expected.size(); i++) {
        ASSERT_EQ(expected.at(i).x / 500, actual.at(i).x);
        ASSERT_EQ(expected.at(i).y / 500, actual.at(i).y);
        ASSERT_EQ(expected.at(i).z / 500, actual.at(i).z);
    }
}

TEST_F(SwingLegTrajectoryGeneratorTest, setPointsScalingTest)
{
    auto new_points = std::vector<Point>();
    auto start_point = Point();
    start_point.x = 0;
    start_point.y = 0;
    start_point.z = 0;

    auto left_point = Point();
    left_point.x = 50;
    left_point.y = 0;
    left_point.z = 100;

    auto right_point = Point();
    right_point.x = 150;
    right_point.y = 0;
    right_point.z = 100;

    auto end_point = Point();
    end_point.x = 200;
    end_point.y = 0;
    end_point.z = 0;

    new_points.push_back(start_point);
    new_points.push_back(left_point);
    new_points.push_back(right_point);
    new_points.push_back(end_point);
    swing_leg_generator->set_points(new_points);

    auto expected = std::vector<Point>();
    start_point = Point();
    start_point.x = 0;
    start_point.y = 0;
    start_point.z = 0;

    left_point = Point();
    left_point.x = 25;
    left_point.y = 0;
    left_point.z = 50;

    right_point = Point();
    right_point.x = 75;
    right_point.y = 0;
    right_point.z = 50;

    end_point = Point();
    end_point.x = 100;
    end_point.y = 0;
    end_point.z = 0;

    expected.push_back(start_point);
    expected.push_back(left_point);
    expected.push_back(right_point);
    expected.push_back(end_point);

    auto actual = swing_leg_generator->get_curve().points;
    ASSERT_EQ(expected.size(), actual.size());
    for (size_t i = 0; i < expected.size(); i++) {
        ASSERT_EQ(expected.at(i).x / 500, actual.at(i).x);
        ASSERT_EQ(expected.at(i).y / 500, actual.at(i).y);
        ASSERT_EQ(expected.at(i).z / 500, actual.at(i).z);
    }
}

// NOLINTEND
#endif