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

#include <memory>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <xsensdeviceapi.h>
#include <xstypes.h>
#include <xsens/xsmutex.h>

#include "march_imu_manager/MasterCallback.h"
#include "march_imu_manager/MtwCallback.h"
#include "march_imu_manager/findClosestUpdateRate.h"

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

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "march_imu_manager");
    ros::NodeHandle node;

    WirelessMasterCallback wirelessMasterCallback;
    std::vector<std::unique_ptr<MtwCallback>> mtwCallbacks;

    XsControl* control = XsControl::construct();
    if (control == 0)
    {
        ROS_ERROR("Failed to construct XsControl instance");
        return -1;
    }

    XsPortInfoArray detectedDevices = XsScanner::scanPorts();
    XsPortInfoArray::const_iterator wirelessMasterPort = detectedDevices.begin();

    ROS_INFO("Scanning for devices...");

    while (wirelessMasterPort != detectedDevices.end() && !wirelessMasterPort->deviceId().isWirelessMaster())
    {
        ++wirelessMasterPort;
    }
    if (wirelessMasterPort == detectedDevices.end())
    {
        ROS_ERROR("No dongle found");
        return -1;
    }

    control->close();

    return 0;
}
