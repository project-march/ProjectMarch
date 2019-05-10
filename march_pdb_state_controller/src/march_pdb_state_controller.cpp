// Copyright 2019 Project March.

#include <march_pdb_state_controller/march_pdb_state_controller.h>

namespace march_pdb_state_controller
{
bool MarchPdbStateController::init(march_hardware_interface::MarchPdbStateInterface* hw, ros::NodeHandle& root_nh,
                                   ros::NodeHandle& controller_nh)
{
  // Get all temperature_sensors from the hardware interface
  const std::vector<std::string>& pdb_state_names = hw->getNames();
  for (unsigned i = 0; i < pdb_state_names.size(); i++)
    ROS_INFO("Got pdb state %s", pdb_state_names[i].c_str());

  // get publishing period
  if (!controller_nh.getParam("publish_rate", publish_rate_))
  {
    ROS_ERROR("Parameter 'publish_rate' not set");
    return false;
  }

  for (unsigned i = 0; i < pdb_state_names.size(); i++)
  {
    // sensor handle
    pdb_state_.push_back(hw->getHandle(pdb_state_names[i]));

    // realtime publisher
    RtPublisherPtr rt_pub(new realtime_tools::RealtimePublisher<sensor_msgs::Temperature>(
        root_nh, "/march/pdb/" + pdb_state_names[i], 4));
    realtime_pubs_.push_back(rt_pub);
  }

  // Last published times
  last_publish_times_.resize(pdb_state_names.size());
  return true;
}

void MarchPdbStateController::starting(const ros::Time& time)
{
  // initialize time
  for (unsigned i = 0; i < last_publish_times_.size(); i++)
  {
    last_publish_times_[i] = time;
  }
}

void MarchPdbStateController::update(const ros::Time& time, const ros::Duration& /*period*/)
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

        // TODO(TIM) Set real message!
        realtime_pubs_[i]->msg_.temperature =
            pdb_state_[i].getPowerDistributionBoard()->getLowVoltage().getNetCurrent(1);

        realtime_pubs_[i]->unlockAndPublish();
      }
    }
  }
}

void MarchPdbStateController::stopping(const ros::Time& /*time*/)
{
}
}  // namespace march_pdb_state_controller

PLUGINLIB_EXPORT_CLASS(march_pdb_state_controller::MarchPdbStateController, controller_interface::ControllerBase)
