/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include "ros/package.h"
#include "rospack/rospack.h"

#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <sstream>
#include <iostream>
#include <fstream>

#include "../include/awindamonitor/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

//namespace awindamonitor {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
{
	init();
}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

void QNode::showNoMasterMessage() {
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();

}

bool QNode::init() {
	ros::init(init_argc,init_argv,"awindamonitor");
	if ( ! ros::master::check() ) {
        showNoMasterMessage();
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	
	//load conf file;
        std::string path = ros::package::getPath("awindamonitor");
        path.append("/cfg/sensor_label.txt");

        std::fstream myfile(path.c_str());

        if(!myfile.is_open()){
                ROS_INFO("ERROR reading file %s", path.c_str());
                exit(0);
        }

	std::string key_sensor, label;
        while (myfile >> key_sensor >> label)
	{
		ros::Publisher pub = n.advertise<sensor_msgs::Imu>(("sensor_" + label), 1000);
		ros::Publisher pub_rpy = n.advertise<geometry_msgs::Vector3>(("sensor_" + label + "_rpy"), 1000);
		publishers[key_sensor] = pub;
		publishers_rpy[key_sensor] = pub_rpy;
	}


	// Add your ros communications here.
	acc_test_publisher = n.advertise<std_msgs::Float64>("/acc_test", 1000);
	//ros::Publisher pub_00B40FB7 = n.advertise<std_msgs::Float64>("/acc_x_sens_1", 1000);
	//ros::Publisher pub_00B4126B = n.advertise<std_msgs::Float64>("/acc_x_sens_2", 1000);

	//publishers["00B40FB7"] =  pub_00B40FB7;
	//publishers["00B4126B"] =  pub_00B4126B;
	//coro_command_publisher = n.advertise<std_msgs::String>("/coro/command", 1000);

    //dcart_status_subscriber = n.subscribe("/dcart/status", 1000, &QNode::callbackDcartStatus, this);
    //coro_status_subscriber = n.subscribe("/coro/status", 1000, &QNode::callbackCoroStatus, this);

	start();
	return true;
}

/*
void QNode::postDcartCommand(QString cmd){
	std_msgs::String msg;
	std::stringstream ss;
	ss << cmd.toStdString();
	msg.data = ss.str();
	dcart_command_publisher.publish(msg);
}
*/

/*
void QNode::postCoroCommand(QString cmd){
    std_msgs::String msg;
    std::stringstream ss;
    ss << cmd.toStdString();
    msg.data = ss.str();
    coro_command_publisher.publish(msg);
}
*/

void QNode::run() {
	ros::Rate loop_rate(10);
    //int count = 0;
	while ( ros::ok() ) {

		ros::spinOnce();
		loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

// ROS CALLBACKS
/*
void QNode::callbackDcartStatus(std_msgs::String msg)
{
    //ROS_INFO("Dcart %s", msg.data.c_str());
    dcart_status = QString(msg.data.c_str());
    Q_EMIT dcartStatusChanged();
}
*/

/*
void QNode::callbackCoroStatus(std_msgs::String msg)
{
    //ROS_INFO("Coro %s", msg.data.c_str());
    coro_status = QString(msg.data.c_str());
    Q_EMIT coroStatusChanged();
}
*/

/*
QString QNode::getDcartStatus(){
    return dcart_status;
}
*/

/*
QString QNode::getCoroStatus(){
    return coro_status;
}
*/

//}  // namespace awindamonitor 
