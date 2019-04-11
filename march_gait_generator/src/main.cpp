// Copyright 2019 Project March

#include <ros/ros.h>
#include <march_gait_generator/GaitGenerator.h>
#include <QApplication>
#include <march_gait_generator/Gait.h>

int main(int argc, char** argv)
{
  if (!ros::isInitialized())
  {
    ros::init(argc, argv, "gaitgenerator");
  }

  Gait testGait = Gait("Dummy gait", "Not a very interesting gait", "0.1");

  QApplication app(argc, argv);

  GaitGenerator* gaitGenerator = new GaitGenerator();
  gaitGenerator->showMaximized();
  gaitGenerator->show();
  app.setStyleSheet("QGroupBox {"
                    "    border: 1px solid gray;"
                    //                      "    border-radius: 9px;"
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
