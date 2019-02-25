#include <unistd.h>

#include <ros/ros.h>

#include <march_hardware/Joint.h>
#include <march_hardware/March4.h>
#include "sensor_msgs/JointState.h"

int main(int argc, char** argv)
{
  march4cpp::MARCH4 march4 = march4cpp::MARCH4();
  march4.startEtherCAT();

  if (!march4.isOperational())
  {
    ROS_FATAL("Ethercat is not operational");
    return 0;
  }

    ros::init(argc, argv, "dummy");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("march/joint_states", 5);


    printf("march4 initialized\n");
  printf("slaveindex: %d\n", march4.getJoint("test_joint").hasIMotionCube());
  for(int i = 0; i< 10000; i++) {
      usleep(100000);
      float angleVal = march4.getJoint("test_joint").getAngle();
      printf("imc get: %f\n", angleVal);
      sensor_msgs::JointState joint_state;
      joint_state.header.stamp = ros::Time::now();
      joint_state.name = {"test_joint"};
      joint_state.position = {angleVal};
      pub.publish(joint_state);
  }

  march4.stopEtherCAT();
}
