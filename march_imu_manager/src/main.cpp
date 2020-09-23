/*  Copyright (c) 2003-2016 Xsens Technologies B.V. or subsidiaries worldwide.
        All rights reserved.

        Redistribution and use in source and binary forms, with or without modification,
        are permitted provided that the following conditions are met:

        1.      Redistributions of source code must retain the above copyright notice,
        this list of conditions and the following disclaimer.

        2.      Redistributions in binary form must reproduce the above copyright notice,
        this list of conditions and the following disclaimer in the documentation
        and/or other materials provided with the distribution.

        3.      Neither the names of the copyright holders nor the names of their contributors
        may be used to endorse or promote products derived from this software without
        specific prior written permission.

        THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
        EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
        MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
        THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
        SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
        OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
        HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
        TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
        SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/////////////////////////////////////////////////////////////////////////
// Adaptation by Leonardo Felipe L. S. dos Santos, 2019 (@qleonardolp) //
/////////////////////////////////////////////////////////////////////////
// Further adapted for the March exoskeleton by Project March 2019

#include <ros/ros.h>

#include "march_imu_manager/wireless_master.h"

/*
   | MTw  | desiredUpdateRate (max) |
   |------|-------------------------|
   |  1   |           150 Hz        |
   |  2   |           120 Hz        |
   |  4   |           100 Hz        |
   |  6   |            75 Hz        |
   |  12  |            50 Hz        |
   |  18  |            40 Hz        |
*/
#define UPDATE_RATE 120
#define RADIO_CHANNEL 25
#define LOOP_RATE 2000

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "march_imu_manager");
  ros::NodeHandle node;

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  WirelessMaster wireless_master(&node);
  int error = wireless_master.init();
  if (error)
  {
    ROS_FATAL_STREAM("Failed to construct wireless master instance");
    return -1;
  }
  ROS_INFO("Found wireless master");

  error = wireless_master.configure(UPDATE_RATE, RADIO_CHANNEL);
  if (error)
  {
    ROS_FATAL_STREAM("Failed to configure wireless master instance");
    return -1;
  }

  // Blocks this thread until 1 MTw has connected
  wireless_master.waitForConnections(1);

  ROS_DEBUG("Starting measurement...");
  if (!wireless_master.startMeasurement())
  {
    ROS_FATAL("Failed to start measurement");
    return -1;
  }

  ros::Rate loop_rate = LOOP_RATE;

  ROS_DEBUG("Publish loop starting...");

  while (ros::ok())
  {
    wireless_master.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
