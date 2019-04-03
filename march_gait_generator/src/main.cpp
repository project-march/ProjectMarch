#include <QApplication>
#include <ros/ros.h>
#include "gaitgenerator.h"

int main(int argc, char **argv)
{
    ros::init( argc, argv, "gait_generator");


    ros::NodeHandle nh;

    QApplication app( argc, argv );

    GaitGenerator* gaitGenerator = new GaitGenerator();

    gaitGenerator->show();




    app.exec();

    delete gaitGenerator;
}
