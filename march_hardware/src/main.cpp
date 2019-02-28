#include <cmath>

#include <unistd.h>

#include <ros/ros.h>

#include <bitset>

#include <march_hardware/Joint.h>
#include <march_hardware/March4.h>
#include "sensor_msgs/JointState.h"
#include <march_hardware/EtherCAT/EthercatIO.h>
#include <march_hardware/EtherCAT/EthercatSDO.h>

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

  march4.getJoint("test_joint").getIMotionCube().goToOperationEnabled();

  ROS_INFO("march4 initialized");
  float encoderValue = march4.getJoint("test_joint").getAngleRad();

  //    march4.getJoint("test_joint").actuateRad(encoderValue-0.03);

  ros::Rate r(100);
  for (int i = 0; i < 10000; i++)
  {
    float sinusDivider = 1000;
    float index = i / sinusDivider;
    float target = static_cast<float>(std::sin(index) * 0.25 + 0.75);
    printf("Target: %f\n", target);
    march4.getJoint("test_joint").actuateRad(target);
    r.sleep();
    ROS_INFO_STREAM_THROTTLE(2, "Angle: " << march4.getJoint("test_joint").getAngleRad());
  }

  int motionError = march4.getJoint("test_joint").getIMotionCube().getMotionError();
  march4.getJoint("test_joint").getIMotionCube().parseMotionError(motionError);
  ROS_WARN_STREAM(
      "DetailedErrorRegister: " << std::bitset<16>(march4.getJoint("test_joint").getIMotionCube().getDetailedError()));
  ROS_WARN_STREAM("Angle: " << march4.getJoint("test_joint").getAngleRad());

  ROS_INFO_STREAM(
      "Status word final: " << std::bitset<16>(march4.getJoint("test_joint").getIMotionCube().getStatusWord()));

  //    angleVal = march4.getJoint("test_joint").getAngleRad();
  //    printf("imc get: %f\n", angleVal);
  //      sensor_msgs::JointState joint_state;
  //      joint_state.header.stamp = ros::Time::now();
  //      joint_state.name = {"test_joint"};
  //      joint_state.position = {angleVal};
  //      pub.publish(joint_state);
  //  }

  march4.stopEtherCAT();
}
