// Copyright 2019 Project March
#include "march_imu_manager/wireless_master.h"

#include <limits>
#include <string>

#include <sensor_msgs/Imu.h>

WirelessMaster::WirelessMaster(ros::NodeHandle* node) : node_(node)
{
  this->control_ = XsControl::construct();
}

WirelessMaster::~WirelessMaster()
{
  if (this->master_)
  {
    ROS_DEBUG("Disabling radio for shutdown...");
    if (!this->master_->gotoConfig())
    {
      ROS_FATAL("Failed to go to config mode");
    }
    if (!this->master_->disableRadio())
    {
      ROS_FATAL_STREAM("Failed to disable radio channel");
    }
  }

  if (this->control_)
  {
    ROS_DEBUG("Closing XsControl...");
    this->control_->close();
    delete control_;
  }
}

int WirelessMaster::init()
{
  ROS_DEBUG("Scanning for wireless masters...");
  XsPortInfoArray detected_devices = XsScanner::scanPorts();
  XsPortInfoArray::const_iterator wireless_master_port = detected_devices.begin();

  while (wireless_master_port != detected_devices.end() && !wireless_master_port->deviceId().isWirelessMaster())
  {
    ++wireless_master_port;
  }
  if (wireless_master_port == detected_devices.end())
  {
    ROS_FATAL("No wireless master found");
    return -1;
  }

  ROS_DEBUG("Found a device with ID: %s @ port: %s, baudrate: %d",
            wireless_master_port->deviceId().toString().toStdString().c_str(),
            wireless_master_port->portName().toStdString().c_str(), wireless_master_port->baudrate());

  if (!this->control_->openPort(wireless_master_port->portName().toStdString(), wireless_master_port->baudrate()))
  {
    ROS_FATAL_STREAM("Failed to open port " << *wireless_master_port);
    return -1;
  }

  ROS_DEBUG("Getting XsDevice instance for wireless master...");
  this->master_ = this->control_->device(wireless_master_port->deviceId());

  ROS_DEBUG("Attaching callback handler for master...");
  this->master_->addCallbackHandler(this);

  return 0;
}

int WirelessMaster::configure(const int update_rate, const int channel)
{
  if (this->master_ && !this->master_->gotoConfig())
  {
    ROS_FATAL("Failed to go to config mode");
    return -1;
  }

  ROS_DEBUG("Getting the list of the supported update rates...");
  const XsIntArray supported_update_rates = this->master_->supportedUpdateRates();

  std::ostringstream update_rates;
  for (const int rate : supported_update_rates)
  {
    update_rates << rate << " ";
  }
  ROS_DEBUG_STREAM("Supported update rates: " << update_rates.str());

  int new_update_rate = findClosestUpdateRate(supported_update_rates, update_rate);
  new_update_rate = 60;
  ROS_DEBUG_STREAM("Setting update rate to " << new_update_rate << " Hz...");
  if (!this->master_->setUpdateRate(new_update_rate))
  {
    ROS_FATAL_STREAM("Failed to set update rate");
    return -1;
  }

  ROS_DEBUG("Disabling radio channel if previously enabled...");
  if (this->master_->isRadioEnabled())
  {
    if (!this->master_->disableRadio())
    {
      ROS_FATAL_STREAM("Failed to disable radio channel");
      return -1;
    }
  }

  ROS_DEBUG_STREAM("Setting radio channel to " << channel << " and enabling radio...");
  if (!this->master_->enableRadio(channel))
  {
    ROS_FATAL_STREAM("Failed to set radio channel");
    return -1;
  }

  return 0;
}

void WirelessMaster::waitForConnections(const size_t connections)
{
  std::unique_lock<std::mutex> lck(this->mutex_);

  auto has_connections = [this, connections] { return this->connected_mtws_.size() == connections; };
  this->cv_.wait(lck, has_connections);
}

bool WirelessMaster::startMeasurement()
{
  return this->master_ && this->master_->gotoMeasurement();
}

bool WirelessMaster::isMeasuring() const
{
  return this->master_ && this->master_->isMeasuring();
}

void WirelessMaster::update()
{
  for (const auto& mtw : this->connected_mtws_)
  {
    if (mtw.second->dataAvailable())
    {
      const XsDataPacket* packet = mtw.second->getOldestPacket();

      if (packet->containsCalibratedData())
      {
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
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

        this->publishers_[mtw.first].publish(imu_msg);
      }

      mtw.second->deleteOldestPacket();
    }
  }
}

void WirelessMaster::onConnectivityChanged(XsDevice* dev, XsConnectivityState new_state)
{
  std::unique_lock<std::mutex> lck(this->mutex_);
  const uint32_t device_id = dev->deviceId().toInt();
  const std::string device_id_string = dev->deviceId().toString().toStdString();
  switch (new_state)
  {
    case XCS_Disconnected:
      ROS_WARN_STREAM("EVENT: MTW Disconnected -> " << device_id_string);
      this->connected_mtws_.erase(device_id);
      this->publishers_.erase(device_id);
      break;
    case XCS_Rejected:
      ROS_WARN_STREAM("EVENT: MTW Rejected -> " << device_id_string);
      this->connected_mtws_.erase(device_id);
      this->publishers_.erase(device_id);
      break;
    case XCS_PluggedIn:
      ROS_INFO_STREAM("EVENT: MTW PluggedIn -> " << device_id_string);
      this->connected_mtws_.erase(device_id);
      this->publishers_.erase(device_id);
      break;
    case XCS_Wireless:
    {
      ROS_INFO_STREAM("EVENT: MTW Connected -> " << device_id_string);
      this->connected_mtws_.insert(std::make_pair(device_id, std::unique_ptr<Mtw>(new Mtw(dev))));

      ros::Publisher publisher = this->node_->advertise<sensor_msgs::Imu>("/march/imu", 10);
      this->publishers_.insert(std::make_pair(device_id, publisher));
      break;
    }
    case XCS_File:
      ROS_INFO_STREAM("EVENT: MTW File -> " << device_id_string);
      this->connected_mtws_.erase(device_id);
      this->publishers_.erase(device_id);
      break;
    case XCS_Unknown:
      ROS_INFO_STREAM("EVENT: MTW Unkown -> " << device_id_string);
      this->connected_mtws_.erase(device_id);
      this->publishers_.erase(device_id);
      break;
    default:
      ROS_ERROR_STREAM("EVENT: MTW Error -> " << device_id_string);
      this->connected_mtws_.erase(device_id);
      this->publishers_.erase(device_id);
      break;
  }
  lck.unlock();
  this->cv_.notify_one();
}

int WirelessMaster::findClosestUpdateRate(const XsIntArray& supported_update_rates, const int desired_update_rate)
{
  int min_distance = std::numeric_limits<int>::max();
  int closest_update_rate = 0;
  for (const int updateRate : supported_update_rates)
  {
    const int distance = std::abs(updateRate - desired_update_rate);

    if (distance < min_distance)
    {
      min_distance = distance;
      closest_update_rate = updateRate;
    }
  }
  return closest_update_rate;
}
