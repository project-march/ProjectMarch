// Copyright 2019 Project March.

#include "gtest/gtest.h"
#include "ros/ros.h"
#include <gmock/gmock.h>
#include <march_gait_generator/Pose.h>


using ::testing::Return;
using ::testing::AtLeast;

class PoseTest : public ::testing::Test
{
protected:
};


TEST_F(PoseTest, TestEquals)
{
    sensor_msgs::JointState jointState;
    jointState.name.push_back("joint1");
    jointState.name.push_back("joint2");
    jointState.position.push_back(0.1);
    jointState.position.push_back(0.2);
    jointState.velocity.push_back(0.3);
    jointState.velocity.push_back(0.4);

    Pose fromJointState = Pose(jointState);
    Pose fromVectors = Pose({"joint1", "joint2"}, {0.1, 0.2}, {0.3, 0.4});

    ASSERT_EQ(fromJointState, fromVectors);
}

TEST_F(PoseTest, TestNotEquals)
{
    sensor_msgs::JointState jointState;
    Pose fromJointState = Pose(jointState);
    Pose fromVectors = Pose({"joint1", "joint2"}, {0.1, 0.2}, {0.3, 0.4});

    ASSERT_NE(fromJointState, fromVectors);
}

TEST_F(PoseTest, TestGetJointPosition)
{
    sensor_msgs::JointState jointState;
    Pose pose = Pose({"joint1", "joint2"}, {0.1, 0.2}, {0.3, 0.4});

    ASSERT_EQ(0.1, pose.getJointPosition("joint1"));
    ASSERT_EQ(0.2, pose.getJointPosition("joint2"));
}

TEST_F(PoseTest, TestGetJointVelocity)
{
    sensor_msgs::JointState jointState;
    Pose pose = Pose({"joint1", "joint2"}, {0.1, 0.2}, {0.3, 0.4});

    ASSERT_EQ(0.3, pose.getJointVelocity("joint1"));
    ASSERT_EQ(0.4, pose.getJointVelocity("joint2"));
}

TEST_F(PoseTest, TestGetNonExistentJoint)
{
    sensor_msgs::JointState jointState;
    Pose pose = Pose({"joint1", "joint2"}, {0.1, 0.2}, {0.3, 0.4});

    ASSERT_EQ(0, pose.getJointVelocity("wrongname"));
}
