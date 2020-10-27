#include "ros/ros.h"


int main(int argc, char** argv) {

  ros::init(argc, argv, "march_acado_mpc_node")
  ros::NodeHandle n;
  ros::Rate rate(10);

  int count = 0;
  while (!n.hasParam("/march/joint_names"))
  {
    ros::Duration(0.5).sleep();
    count++;
    if (count > 10)
    {
      ROS_ERROR("Failed to read the joint_names from the parameter server.");
      throw std::runtime_error("Failed to read the joint_names from the parameter server.");
    }
  }

  n.getParam("/march/joint_names", sensor_names);

  while (ros::ok())
  {
  	// ...
    
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}