#include <unistd.h>

#include <ros/ros.h>

#include <march_hardware/Joint.h>
#include <march_hardware/March4.h>

int main(int argc, char** argv)
{
  MARCH4 march4 = MARCH4();
  march4.startEtherCAT();

  if (!march4.isOperational())
  {
    ROS_FATAL("Ethercat is not operational");
    return 0;
  }

  printf("march4 initialized\n");
  printf("%f\n", march4.getJoint("TEMP").getTemperature());
  sleep(2);
  march4.sendData(3);
  sleep(2);
  printf("%f\n", march4.getJoint("TEMP").getTemperature());
  sleep(2);
  march4.sendData(5);
  sleep(2);
  printf("%f\n", march4.getJoint("TEMP").getTemperature());

  march4.stopEtherCAT();
}
