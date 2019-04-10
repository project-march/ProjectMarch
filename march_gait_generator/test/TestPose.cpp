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
    Pose pose = Pose({"joint1", "joint2"}, {0.1, 0.2}, {0.3, 0.4});

    ASSERT_EQ(0.1, pose.getJointPosition("joint1"));
    ASSERT_EQ(0.2, pose.getJointPosition("joint2"));
}

TEST_F(PoseTest, TestGetJointVelocity)
{
    Pose pose = Pose({"joint1", "joint2"}, {0.1, 0.2}, {0.3, 0.4});

    ASSERT_EQ(0.3, pose.getJointVelocity("joint1"));
    ASSERT_EQ(0.4, pose.getJointVelocity("joint2"));
}

TEST_F(PoseTest, TestGetNonExistentJoint)
{
    Pose pose = Pose({"joint1", "joint2"}, {0.1, 0.2}, {0.3, 0.4});

    ASSERT_EQ(0, pose.getJointVelocity("wrongname"));
}

TEST_F(PoseTest, TestFromJointState)
{
    sensor_msgs::JointState jointState;
    jointState.name.push_back("joint1");
    jointState.name.push_back("joint2");
    jointState.position.push_back(0.1);
    jointState.position.push_back(0.2);
    jointState.velocity.push_back(0.3);
    jointState.velocity.push_back(0.4);

    Pose fromJointState = Pose(jointState);
    Pose pose = Pose({"joint1", "joint2"}, {0.1, 0.2}, {0.3, 0.4});

    ASSERT_EQ(pose, fromJointState);

    sensor_msgs::JointState newJointState;
    newJointState.name.push_back("joint3");
    newJointState.name.push_back("joint4");
    newJointState.position.push_back(1);
    newJointState.position.push_back(2);
    newJointState.velocity.push_back(3);
    newJointState.velocity.push_back(4);

    pose.fromJointState(newJointState);
    Pose fromNewJointState = Pose(newJointState);

    ASSERT_NE(pose, fromJointState);
    ASSERT_EQ(pose, newJointState);
}

TEST_F(PoseTest, TestToJointStateEqual)
{
    sensor_msgs::JointState jointState;
    jointState.name.push_back("joint1");
    jointState.name.push_back("joint2");
    jointState.position.push_back(0.1);
    jointState.position.push_back(0.2);
    jointState.velocity.push_back(0.3);
    jointState.velocity.push_back(0.4);

    Pose pose = Pose({"joint1", "joint2"}, {0.1, 0.2}, {0.3, 0.4});

    ASSERT_EQ(pose.toJointState().name, jointState.name);
    ASSERT_EQ(pose.toJointState().position, jointState.position);
    ASSERT_EQ(pose.toJointState().velocity, jointState.velocity);
}

TEST_F(PoseTest, TestToJointStateNotEqual)
{
    sensor_msgs::JointState jointState;
    jointState.name.push_back("joint3");
    jointState.name.push_back("joint2");
    jointState.position.push_back(0.1);
    jointState.position.push_back(0.3);
    jointState.velocity.push_back(0.2);
    jointState.velocity.push_back(0.4);

    Pose pose = Pose({"joint1", "joint2"}, {0.1, 0.2}, {0.3, 0.4});

    ASSERT_NE(pose.toJointState().name, jointState.name);
    ASSERT_NE(pose.toJointState().position, jointState.position);
    ASSERT_NE(pose.toJointState().velocity, jointState.velocity);
}