//
// Created by tim on 25-10-18.
//

#include "master_node.h"
#include "std_msgs/Bool.h"
#include "ros/ros.h"
#include <march_custom_msgs/GaitInstruction.h>

enum GaitTypes { Walk, Sit, Stand };

bool gait_instruction(march_custom_msgs::GaitInstruction) {

  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "master_node");
  ros::NodeHandle n;
//  ros::ServiceServer service = n.advertiseService("gait_instructions", gait_instruction);
  ros::spin();
  return 0;
}