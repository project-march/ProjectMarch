#include <unistd.h>

#include <ros/ros.h>

#include <march_hardware/Joint.h>
#include <march_hardware/March4.h>

int main(int argc, char** argv)
{
  march4cpp::MARCH4 march4 = march4cpp::MARCH4();
  march4.startEtherCAT();

  if (!march4.isOperational())
  {
    ROS_FATAL("Ethercat is not operational");
    return 0;
  }

  printf("march4 initialized\n");
  printf("slaveindex: %d\n", march4.getJoint("test_joint").hasIMotionCube());
  for(int i = 0; i< 1; i++) {
      usleep(100000);
//      printf("imc get: %f\n", march4.getJoint("test_joint").getAngle());
  }

  march4.stopEtherCAT();
}
