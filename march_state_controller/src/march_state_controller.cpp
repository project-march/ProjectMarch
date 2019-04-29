// Copyright 2019 Project March.

#include "march_state_controller/march_state_controller.h"

namespace march_state_controller
{
bool MarchStateController::init(march_hardware_interface::MarchStateInterface* hw, ros::NodeHandle& root_nh,
                                ros::NodeHandle& controller_nh)
{
  // get all joint states from the hardware interface
  const std::vector<std::string>& sensor_names = hw->getNames();
  for (unsigned i = 0; i < sensor_names.size(); i++)
    ROS_INFO("Got sensor %s", sensor_names[i].c_str());

  // get publishing period
  if (!controller_nh.getParam("publish_rate", publish_rate_))
  {
    ROS_ERROR("Parameter 'publish_rate' not set");
    return false;
  }

  for (unsigned i = 0; i < sensor_names.size(); i++)
  {
    // sensor handle
    sensors_.push_back(hw->getHandle(sensor_names[i]));

    // realtime publisher
    RtPublisherPtr rt_pub(new realtime_tools::RealtimePublisher<sensor_msgs::Imu>(root_nh, sensor_names[i], 4));
    realtime_pubs_.push_back(rt_pub);
  }

  // Last published times
  last_publish_times_.resize(sensor_names.size());
  return true;
}

void MarchStateController::starting(const ros::Time& time)
{
  // initialize time
  for (unsigned i = 0; i < last_publish_times_.size(); i++)
  {
    last_publish_times_[i] = time;
  }
}

void MarchStateController::update(const ros::Time& time, const ros::Duration& /*period*/)
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
        realtime_pubs_[i]->msg_.header.frame_id = sensors_[i].getFrameId();


        realtime_pubs_[i]->unlockAndPublish();
      }
    }
  }
}

void MarchStateController::stopping(const ros::Time& /*time*/)
{
}
}  // namespace march_state_controller

PLUGINLIB_EXPORT_CLASS(march_state_controller::MarchStateController, controller_interface::ControllerBase)
