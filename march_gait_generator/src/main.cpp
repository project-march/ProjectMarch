// Copyright 2019 Project March

#include <ros/ros.h>
#include <march_gait_generator/GaitGenerator.h>
#include <QApplication>
#include <QtCharts>
#include <QSplineSeries>
#include <march_gait_generator/Gait.h>
#include <march_gait_generator/TrajectoryPreview.h>


int main(int argc, char** argv)
{
  if (!ros::isInitialized())
  {
    ros::init(argc, argv, "gaitgenerator");
  }


  QApplication app(argc, argv);

    Gait testGait = Gait("Dummy gait", "Not a very interesting gait", "0.1");


    std::vector<std::string> joints = {"left_hip", "right_hip"};
    Pose pose1 = Pose(joints, {0.1 ,0.1}, {0 ,0});
    Pose pose2 = Pose(joints, {0.1 ,0.1}, {0 ,0});

    PoseStamped testPose1 = PoseStamped(0.1, 0.1, pose1);
    PoseStamped testPose2 = PoseStamped(0.5, 0.5, pose2);

    testGait.addPoseStamped(testPose1);
    testGait.addPoseStamped(testPose2);

    TrajectoryPreview preview = TrajectoryPreview(testGait);

    QChart *chart = new QChart();
    chart->legend()->hide();
    chart->addSeries(series);
    chart->setTitle("Simple spline chart example");
    chart->createDefaultAxes();
    chart->axes(Qt::Vertical).first()->setRange(0, 10);

    QSplineSeries* series = new QSplineSeries();
    series->append(0, 6);
    series->append(2, 4);
    chart->addSeries(series);

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
