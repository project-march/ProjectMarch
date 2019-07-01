// Copyright 2019 Project March.

#include "march_hardware_state_controller/march_hardware_state_controller.h"

namespace march_hardware_state_controller {
bool MarchHardwareStateController::init(march_hardware_interface::MarchHardwareStateInterface *hw,
                                        ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
  // get publishing period
  if (!controller_nh.getParam("publish_rate", publish_rate_)) {
    ROS_ERROR("Parameter 'publish_rate' not set");
    return false;
  }

  // sensor handle
//  hardware_states_.push_back(hw->getHandle(hardware_state_names[i]));

  // realtime publisher
  RtPublisherPtr rt_pub(new realtime_tools::RealtimePublisher<std_msgs::Empty>(
      root_nh, "/march/example/", 4));
  realtime_pubs_.push_back(rt_pub);


  // Last published times
//  last_publish_times_.resize(hardware_state_names.size());
  return true;
}

void MarchHardwareStateController::starting(const ros::Time &time) {
  // initialize time
  for (unsigned i = 0; i < last_publish_times_.size(); i++) {
    last_publish_times_[i] = time;
  }
}

void MarchHardwareStateController::update(const ros::Time &time, const ros::Duration & /*period*/) {
  // limit rate of publishing
  for (unsigned i = 0; i < realtime_pubs_.size(); i++) {
    if (publish_rate_ > 0.0 && last_publish_times_[i] + ros::Duration(1.0 / publish_rate_) < time) {
      // try to publish
      if (realtime_pubs_[i]->trylock()) {
        // we're actually publishing, so increment time
        last_publish_times_[i] = last_publish_times_[i] + ros::Duration(1.0 / publish_rate_);

        realtime_pubs_[i]->unlockAndPublish();
      }
    }
  }
}

void MarchHardwareStateController::stopping(const ros::Time & /*time*/) {
}
}  // namespace march_hardware_state_controller

PLUGINLIB_EXPORT_CLASS(march_hardware_state_controller::MarchHardwareStateController,
    controller_interface::ControllerBase
)
