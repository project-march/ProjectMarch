// Copyright 2019 Project March
#include <ros/ros.h>

#include "march_imu_manager/WirelessMaster.h"

WirelessMaster::WirelessMaster()
{
    this->m_control = XsControl::construct();
}

WirelessMaster::~WirelessMaster()
{
    ROS_DEBUG("Disabling radio for shutdown...");
    if (!this->m_master->gotoConfig())
    {
        ROS_FATAL("Failed to go to config mode");
    }
    if (!this->m_master->disableRadio())
    {
        ROS_FATAL_STREAM("Failed to disable radio channel");
    }

    this->m_control->close();

    delete m_control;
}

int WirelessMaster::init()
{
    XsPortInfoArray detectedDevices = XsScanner::scanPorts();
    XsPortInfoArray::const_iterator wirelessMasterPort = detectedDevices.begin();

    ROS_DEBUG("Scanning for dongles...");

    while (wirelessMasterPort != detectedDevices.end() && !wirelessMasterPort->deviceId().isWirelessMaster())
    {
        ++wirelessMasterPort;
    }
    if (wirelessMasterPort == detectedDevices.end())
    {
        ROS_FATAL("No dongle found");
        return -1;
    }

    ROS_DEBUG("Found a device with ID: %s @ port: %s, baudrate: %d",
            wirelessMasterPort->deviceId().toString().toStdString().c_str(),
            wirelessMasterPort->portName().toStdString().c_str(), wirelessMasterPort->baudrate());

    if (!this->m_control->openPort(wirelessMasterPort->portName().toStdString(), wirelessMasterPort->baudrate()))
    {
        ROS_FATAL_STREAM("Failed to open port " << *wirelessMasterPort);
        return -1;
    }

    ROS_DEBUG("Getting XsDevice instance for wireless master...");
    this->m_master = this->m_control->device(wirelessMasterPort->deviceId());

    ROS_DEBUG("Attaching callback handler for master...");
    this->m_master->addCallbackHandler(this);

    return 0;
}

int WirelessMaster::configure(const int updateRate, const int channel)
{
    if (!this->m_master->gotoConfig())
    {
        ROS_FATAL("Failed to go to config mode");
        return -1;
    }

    ROS_DEBUG("Getting the list of the supported update rates...");
    const XsIntArray supportedUpdateRates = this->m_master->supportedUpdateRates();

    std::ostringstream updateRates;
    for (const int updateRate : supportedUpdateRates)
    {
        updateRates << updateRate << " ";
    }
    ROS_DEBUG_STREAM("Supported update rates: " << updateRates.str());

    const int newUpdateRate = findClosestUpdateRate(supportedUpdateRates, updateRate);

    ROS_DEBUG_STREAM("Setting update rate to " << newUpdateRate << " Hz...");
    if (!this->m_master->setUpdateRate(newUpdateRate))
    {
        ROS_FATAL_STREAM("Failed to set update rate");
        return -1;
    }

    ROS_DEBUG("Disabling radio channel if previously enabled...");
    if (this->m_master->isRadioEnabled())
    {
        if (!this->m_master->disableRadio())
        {
            ROS_FATAL_STREAM("Failed to disable radio channel");
            return -1;
        }
    }

    ROS_DEBUG_STREAM("Setting radio channel to " << channel << " and enabling radio...");
    if (!this->m_master->enableRadio(channel))
    {
        ROS_FATAL_STREAM("Failed to set radio channel");
        return -1;
    }

    return 0;
}

XsDeviceSet WirelessMaster::getWirelessMTWs() const
{
    XsMutexLocker lock(this->m_mutex);
    return this->m_connectedMTWs;
}

void WirelessMaster::onConnectivityChanged(XsDevice* dev, XsConnectivityState newState)
{
    XsMutexLocker lock(this->m_mutex);
    switch (newState)
    {
        case XCS_Disconnected:
            ROS_INFO_STREAM("EVENT: MTW Disconnected -> " << dev->deviceId().toString().toStdString() );
            this->m_connectedMTWs.erase(dev);
            break;
        case XCS_Rejected:
            ROS_INFO_STREAM("EVENT: MTW Rejected -> " << dev->deviceId().toString().toStdString() );
            this->m_connectedMTWs.erase(dev);
            break;
        case XCS_PluggedIn:
            ROS_INFO_STREAM("EVENT: MTW PluggedIn -> " << dev->deviceId().toString().toStdString() );
            this->m_connectedMTWs.erase(dev);
            break;
        case XCS_Wireless:
            ROS_INFO_STREAM("EVENT: MTW Connected -> " << dev->deviceId().toString().toStdString() );
            this->m_connectedMTWs.insert(dev);
            break;
        case XCS_File:
            ROS_INFO_STREAM("EVENT: MTW File -> " << dev->deviceId().toString().toStdString() );
            this->m_connectedMTWs.erase(dev);
            break;
        case XCS_Unknown:
            ROS_INFO_STREAM("EVENT: MTW Unkown -> " << dev->deviceId().toString().toStdString() );
            this->m_connectedMTWs.erase(dev);
            break;
        default:
            ROS_INFO_STREAM("EVENT: MTW Error -> " << dev->deviceId().toString().toStdString() );
            this->m_connectedMTWs.erase(dev);
            break;
    }
}

int WirelessMaster::findClosestUpdateRate(const XsIntArray& supportedUpdateRates, const int desiredUpdateRate)
{
    if (supportedUpdateRates.empty())
    {
        return 0;
    }

    if (supportedUpdateRates.size() == 1)
    {
        return supportedUpdateRates[0];
    }

    int uRateDist = -1;
    int closestUpdateRate = -1;
    for (const int updateRate : supportedUpdateRates)
    {
        const int currDist = std::abs(updateRate - desiredUpdateRate);

        if ((uRateDist == -1) || (currDist < uRateDist))
        {
            uRateDist = currDist;
            closestUpdateRate = updateRate;
        }
    }
    return closestUpdateRate;
}
