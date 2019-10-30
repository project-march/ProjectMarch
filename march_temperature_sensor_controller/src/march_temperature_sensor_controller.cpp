// Copyright 2019 Project March.

#include "march_temperature_sensor_controller/march_temperature_sensor_controller.h"

namespace march_temperature_sensor_controller
{
bool MarchTemperatureSensorController::init(march_hardware_interface::MarchTemperatureSensorInterface* hw,
                                            ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  // get all temperature_sensors from the hardware interface
  const std::vector<std::string>& temperature_sensor_names = hw->getNames();
  for (unsigned i = 0; i < temperature_sensor_names.size(); i++)
  {
    ROS_DEBUG("Got temperature sensor %s", temperature_sensor_names[i].c_str());
  }

  // get publishing period
  if (!controller_nh.getParam("publish_rate", publish_rate_))
  {
    ROS_ERROR("Parameter 'publish_rate' not set");
    return false;
  }

  for (unsigned i = 0; i < temperature_sensor_names.size(); i++)
  {
    // sensor handle
    temperature_sensors_.push_back(hw->getHandle(temperature_sensor_names[i]));

    // realtime publisher
    RtPublisherPtr rt_pub(new realtime_tools::RealtimePublisher<sensor_msgs::Temperature>(
        root_nh, "/march/temperature/" + temperature_sensor_names[i], 4));
    realtime_pubs_.push_back(rt_pub);
  }

  // Last published times
  last_publish_times_.resize(temperature_sensor_names.size());
  return true;
}

void MarchTemperatureSensorController::starting(const ros::Time& time)
{
  // initialize time
  for (unsigned i = 0; i < last_publish_times_.size(); i++)
  {
    last_publish_times_[i] = time;
  }
}

void MarchTemperatureSensorController::update(const ros::Time& time, const ros::Duration& /*period*/)
{
  // limit rate of publishing
  for (unsigned i = 0; i < realtime_pubs_.size(); i++)
  {
    if (publish_rate_ > 0.0 && last_publish_times_[i] + ros::Duration(1.0 / publish_rate_) < time)
    {
      // try to publish
      if (realtime_pubs_[i]->trylock())
      {
        // we're actually publishing, so increment time
        last_publish_times_[i] = last_publish_times_[i] + ros::Duration(1.0 / publish_rate_);

        // populate message
        realtime_pubs_[i]->msg_.header.stamp = time;

        realtime_pubs_[i]->msg_.temperature = *temperature_sensors_[i].getTemperature();
        realtime_pubs_[i]->msg_.variance = *temperature_sensors_[i].getVariance();

        realtime_pubs_[i]->unlockAndPublish();
      }
    }
  }
}

void MarchTemperatureSensorController::stopping(const ros::Time& /*time*/)
{
}
}  // namespace march_temperature_sensor_controller

PLUGINLIB_EXPORT_CLASS(march_temperature_sensor_controller::MarchTemperatureSensorController,
                       controller_interface::ControllerBase)
