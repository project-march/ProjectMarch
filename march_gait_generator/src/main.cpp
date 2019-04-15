// Copyright 2019 Project March

#include <ros/ros.h>
#include <march_gait_generator/GaitGenerator.h>
#include <QApplication>
#include <march_gait_generator/Gait.h>
#include <march_gait_generator/TrajectoryPreview.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <std_msgs/Empty.h>
#include <actionlib/client/simple_action_client.h>

int main(int argc, char** argv)
{
  if (!ros::isInitialized())
  {
    ros::init(argc, argv, "gaitgenerator");
  }

  QApplication app(argc, argv);

  Gait testGait = Gait("Dummy gait", "Not a very interesting gait", "0.1");

  std::vector<std::string> joints = { "test_joint",};
  Pose pose1 = Pose(joints, { 0.6}, { 0});

  PoseStamped testPose1 = PoseStamped(0.1, 0.1, pose1);

  testGait.addPoseStamped(testPose1);

  ros::NodeHandle n;

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> actionClient("/march/controller/trajectory/testjoint/follow_joint_trajectory/");

    control_msgs::FollowJointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints

    // We will have two waypoints in this goal trajectory
    goal.trajectory = testGait.toJointTrajectory();

    // First trajectory point
    // Positions

    ROS_INFO_STREAM(goal);

    actionClient.sendGoal(goal);
    ROS_INFO("Action state: %s",actionClient.getState().toString().c_str());

    bool finished_before_timeout = actionClient.waitForResult(ros::Duration(5.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = actionClient.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else {
        ROS_INFO("Action did not finish before the time out.");
    }

    ROS_INFO("Action state: %s",actionClient.getState().toString().c_str());

//  TrajectoryPreview preview = TrajectoryPreview(testGait);

  //  GaitGenerator* gaitGenerator = new GaitGenerator();
  //  gaitGenerator->showMaximized();
  //  gaitGenerator->show();
  //  app.setStyleSheet("QGroupBox {"
  //                    "    border: 1px solid gray;"
  //                                          "    border-radius: 9px;"
  //                    "    margin-top: 0.5em;"
  //                    "}"
  //                    ""
  //                    "QGroupBox::title {"
  //                    "    subcontrol-origin: margin;"
  //                    "    left: 10px;"
  //                    "    padding: 0 3px 0 3px;"
  //                    "}");
  //
  //  app.exec();
  //
  //  delete gaitGenerator;
}
