// Copyright 2019 Project March
#include "march_imu_manager/WirelessMaster.h"

#include <limits>
#include <string>

#include <sensor_msgs/Imu.h>

WirelessMaster::WirelessMaster(ros::NodeHandle* node) :
    m_node(node)
{
    this->m_control = XsControl::construct();
}

WirelessMaster::~WirelessMaster()
{
    if (this->m_master)
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
    }

    if (this->m_control)
    {
        ROS_DEBUG("Closing XsControl...");
        this->m_control->close();
        delete m_control;
    }
}

int WirelessMaster::init()
{
    ROS_DEBUG("Scanning for wireless masters...");
    XsPortInfoArray detectedDevices = XsScanner::scanPorts();
    XsPortInfoArray::const_iterator wirelessMasterPort = detectedDevices.begin();

    while (wirelessMasterPort != detectedDevices.end() && !wirelessMasterPort->deviceId().isWirelessMaster())
    {
        ++wirelessMasterPort;
    }
    if (wirelessMasterPort == detectedDevices.end())
    {
        ROS_FATAL("No wireless master found");
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
    if (this->m_master && !this->m_master->gotoConfig())
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

void WirelessMaster::waitForConnections(const size_t connections)
{
    std::unique_lock<std::mutex> lck(this->m_mutex);

    auto hasConnections = [this, connections]
    {
        return this->m_connectedMtws.size() == connections;
    };
    this->m_cv.wait(lck, hasConnections);
}

bool WirelessMaster::startMeasurement()
{
    return this->m_master && this->m_master->gotoMeasurement();
}

bool WirelessMaster::isMeasuring() const
{
    return this->m_master && this->m_master->isMeasuring();
}

void WirelessMaster::update()
{
    for (const auto& mtw : this->m_connectedMtws)
    {
        if (mtw.second->dataAvailable())
        {
            const XsDataPacket* packet = mtw.second->getOldestPacket();

            if (packet->containsCalibratedData())
            {
                sensor_msgs::Imu imu_msg;
                imu_msg.header.frame_id = "imu_link";

                // [m/s²]
                imu_msg.linear_acceleration.x = packet->calibratedAcceleration().value(0);
                imu_msg.linear_acceleration.y = packet->calibratedAcceleration().value(1);
                imu_msg.linear_acceleration.z = packet->calibratedAcceleration().value(2);
                imu_msg.linear_acceleration_covariance[0] = -1;

                // [rad/s]
                imu_msg.angular_velocity.x = packet->calibratedGyroscopeData().value(0);
                imu_msg.angular_velocity.y = packet->calibratedGyroscopeData().value(1);
                imu_msg.angular_velocity.z = packet->calibratedGyroscopeData().value(2);
                imu_msg.angular_velocity_covariance[0] = -1;

                // unit quaternion
                imu_msg.orientation.x = packet->orientationQuaternion().x();
                imu_msg.orientation.y = packet->orientationQuaternion().y();
                imu_msg.orientation.z = packet->orientationQuaternion().z();
                imu_msg.orientation.w = packet->orientationQuaternion().w();
                imu_msg.orientation_covariance[0] = -1;

                this->m_publishers[mtw.first].publish(imu_msg);
            }

            mtw.second->deleteOldestPacket();
        }
    }
}

void WirelessMaster::onConnectivityChanged(XsDevice* dev, XsConnectivityState newState)
{
    std::unique_lock<std::mutex> lck(this->m_mutex);
    const uint32_t deviceId = dev->deviceId().toInt();
    const std::string deviceIdString = dev->deviceId().toString().toStdString();
    switch (newState)
    {
        case XCS_Disconnected:
            ROS_WARN_STREAM("EVENT: MTW Disconnected -> " << deviceIdString);
            this->m_connectedMtws.erase(deviceId);
            this->m_publishers.erase(deviceId);
            break;
        case XCS_Rejected:
            ROS_WARN_STREAM("EVENT: MTW Rejected -> " << deviceIdString);
            this->m_connectedMtws.erase(deviceId);
            this->m_publishers.erase(deviceId);
            break;
        case XCS_PluggedIn:
            ROS_INFO_STREAM("EVENT: MTW PluggedIn -> " << deviceIdString);
            this->m_connectedMtws.erase(deviceId);
            this->m_publishers.erase(deviceId);
            break;
        case XCS_Wireless:
            {
                ROS_INFO_STREAM("EVENT: MTW Connected -> " << deviceIdString);
                this->m_connectedMtws.insert(std::make_pair(deviceId, std::unique_ptr<Mtw>(new Mtw(dev))));

                ros::Publisher publisher =
                    this->m_node->advertise<sensor_msgs::Imu>("march/imu/", 10);
                this->m_publishers.insert(std::make_pair(deviceId, publisher));
                break;
            }
        case XCS_File:
            ROS_INFO_STREAM("EVENT: MTW File -> " << deviceIdString);
            this->m_connectedMtws.erase(deviceId);
            this->m_publishers.erase(deviceId);
            break;
        case XCS_Unknown:
            ROS_INFO_STREAM("EVENT: MTW Unkown -> " << deviceIdString);
            this->m_connectedMtws.erase(deviceId);
            this->m_publishers.erase(deviceId);
            break;
        default:
            ROS_ERROR_STREAM("EVENT: MTW Error -> " << deviceIdString);
            this->m_connectedMtws.erase(deviceId);
            this->m_publishers.erase(deviceId);
            break;
    }
    lck.unlock();
    this->m_cv.notify_one();
}

int WirelessMaster::findClosestUpdateRate(const XsIntArray& supportedUpdateRates, const int desiredUpdateRate)
{
    int minDistance = std::numeric_limits<int>::max();
    int closestUpdateRate = 0;
    for (const int updateRate : supportedUpdateRates)
    {
        const int distance = std::abs(updateRate - desiredUpdateRate);

        if (distance < minDistance)
        {
            minDistance = distance;
            closestUpdateRate = updateRate;
        }
    }
    return closestUpdateRate;
}
