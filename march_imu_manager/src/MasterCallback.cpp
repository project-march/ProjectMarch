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
        case XCS_Disconnected:          /*!< Device has disconnected, only limited informational functionality is available. */

            //std::cout << "\nEVENT: MTW Disconnected -> " << *dev << std::endl;
            ROS_INFO_STREAM("EVENT: MTW Disconnected -> " << dev->deviceId().toString().toStdString() );
            m_connectedMTWs.erase(dev);
            break;
        case XCS_Rejected:                      /*!< Device has been rejected and is disconnected, only limited informational functionality is available. */
            ROS_INFO_STREAM("EVENT: MTW Rejected -> " << dev->deviceId().toString().toStdString() );
            m_connectedMTWs.erase(dev);
            break;
        case XCS_PluggedIn:                     /*!< Device is connected through a cable. */
            ROS_INFO_STREAM("EVENT: MTW PluggedIn -> " << dev->deviceId().toString().toStdString() );
            m_connectedMTWs.erase(dev);
            break;
        case XCS_Wireless:                      /*!< Device is connected wirelessly. */
            ROS_INFO_STREAM("EVENT: MTW Connected -> " << dev->deviceId().toString().toStdString() );
            m_connectedMTWs.insert(dev);
            break;
        case XCS_File:                          /*!< Device is reading from a file. */
            ROS_INFO_STREAM("EVENT: MTW File -> " << dev->deviceId().toString().toStdString() );
            m_connectedMTWs.erase(dev);
            break;
        case XCS_Unknown:                       /*!< Device is in an unknown state. */
            ROS_INFO_STREAM("EVENT: MTW Unkown -> " << dev->deviceId().toString().toStdString() );
            m_connectedMTWs.erase(dev);
            break;
        default:
            ROS_INFO_STREAM("EVENT: MTW Error -> " << dev->deviceId().toString().toStdString() );
            m_connectedMTWs.erase(dev);
            break;
    }
}
