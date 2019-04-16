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

    GaitGenerator* gaitGenerator = new GaitGenerator();
    gaitGenerator->showMaximized();
    gaitGenerator->show();
    app.setStyleSheet("QGroupBox {"
                      "    border: 1px solid gray;"
                                            "    border-radius: 9px;"
                      "    margin-top: 0.5em;"
                      "}"
                      ""
                      "QGroupBox::title {"
                      "    subcontrol-origin: margin;"
                      "    left: 10px;"
                      "    padding: 0 3px 0 3px;"
                      "}");

    app.exec();

    delete gaitGenerator;
}
