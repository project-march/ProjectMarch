// Copyright 2019 Project March
#include <ros/ros.h>

#include "march_imu_manager/MasterCallback.h"

XsDeviceSet WirelessMasterCallback::getWirelessMTWs() const
{
    XsMutexLocker lock(m_mutex);
    return m_connectedMTWs;
}

void WirelessMasterCallback::onConnectivityChanged(XsDevice* dev, XsConnectivityState newState)
{
    XsMutexLocker lock(m_mutex);
    switch (newState)
    {
        case XCS_Disconnected:
            ROS_INFO_STREAM("EVENT: MTW Disconnected -> " << dev->deviceId().toString().toStdString() );
            m_connectedMTWs.erase(dev);
            break;
        case XCS_Rejected:
            ROS_INFO_STREAM("EVENT: MTW Rejected -> " << dev->deviceId().toString().toStdString() );
            m_connectedMTWs.erase(dev);
            break;
        case XCS_PluggedIn:
            ROS_INFO_STREAM("EVENT: MTW PluggedIn -> " << dev->deviceId().toString().toStdString() );
            m_connectedMTWs.erase(dev);
            break;
        case XCS_Wireless:
            ROS_INFO_STREAM("EVENT: MTW Connected -> " << dev->deviceId().toString().toStdString() );
            m_connectedMTWs.insert(dev);
            break;
        case XCS_File:
            ROS_INFO_STREAM("EVENT: MTW File -> " << dev->deviceId().toString().toStdString() );
            m_connectedMTWs.erase(dev);
            break;
        case XCS_Unknown:
            ROS_INFO_STREAM("EVENT: MTW Unkown -> " << dev->deviceId().toString().toStdString() );
            m_connectedMTWs.erase(dev);
            break;
        default:
            ROS_INFO_STREAM("EVENT: MTW Error -> " << dev->deviceId().toString().toStdString() );
            m_connectedMTWs.erase(dev);
            break;
    }
}
