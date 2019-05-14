// Copyright 2019 Project March.

#include <march_pdb_state_controller/march_pdb_state_controller.h>

namespace march_pdb_state_controller {
bool MarchPdbStateController::init(
    march_hardware_interface::MarchPdbStateInterface *hw,
    ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
  ROS_INFO("init MarchPdbStateController");

  // Get all temperature_sensors from the hardware interface
  const std::vector<std::string> &pdb_state_names = hw->getNames();
  for (unsigned i = 0; i < pdb_state_names.size(); i++)
    ROS_INFO("Got pdb state %s", pdb_state_names[i].c_str());

  ROS_INFO("amount of pdb_state_names: %zu", pdb_state_names.size());
  // get publishing period
  if (!controller_nh.getParam("publish_rate", publish_rate_)) {
    ROS_ERROR("Parameter 'publish_rate' not set");
    return false;
  }

  for (unsigned i = 0; i < pdb_state_names.size(); i++) {
    // sensor handle
    pdb_state_.push_back(hw->getHandle(pdb_state_names[i]));

    // realtime publisher
    RtPublisherPtr rt_pub(new realtime_tools::RealtimePublisher<
                          march_shared_resources::PowerDistributionBoardState>(
        root_nh, "/march/pdb/" + pdb_state_names[i], 4));
    realtime_pubs_.push_back(rt_pub);
  }

  // Last published times
  last_publish_times_.resize(pdb_state_names.size());
  return true;
}

void MarchPdbStateController::starting(const ros::Time &time) {
  ROS_INFO("starting MarchPdbStateController");
  // initialize time
  for (unsigned i = 0; i < last_publish_times_.size(); i++) {
    last_publish_times_[i] = time;
  }
}
march_shared_resources::PowerNet MarchPdbStateController::createPowerNetMessage(
    march4cpp::HighVoltage high_voltage) {
  march_shared_resources::PowerNet power_net_msg;
  for (int i = 1; i < 9; i++) {
    power_net_msg.net_names.push_back(boost::lexical_cast<std::string>(i));
    // TODO(TIM) There is no net current for each high voltage net
    power_net_msg.net_currents.push_back(high_voltage.getNetCurrent());
    power_net_msg.net_operational.push_back(high_voltage.getNetOperational(i));
    power_net_msg.net_overcurrent_triggered.push_back(
        high_voltage.getOvercurrentTrigger(i));
  }
  power_net_msg.emergency_button_triggered =
      high_voltage.getEmergencyButtonTrigger();
  return power_net_msg;
}
march_shared_resources::PowerNet MarchPdbStateController::createPowerNetMessage(
    march4cpp::LowVoltage low_voltage) {
  march_shared_resources::PowerNet power_net_msg;
  for (int i = 1; i < 3; i++) {
    power_net_msg.net_names.push_back(boost::lexical_cast<std::string>(i));
    power_net_msg.net_currents.push_back(low_voltage.getNetCurrent(i));
    power_net_msg.net_operational.push_back(low_voltage.getNetOperational(i));
    power_net_msg.net_overcurrent_triggered.push_back(false);
  }
  power_net_msg.emergency_button_triggered = false;
  return power_net_msg;
}

void MarchPdbStateController::update(const ros::Time &time,
                                     const ros::Duration & /*period*/) {
  ROS_INFO_THROTTLE(10, "update MarchPdbStateController");
  ROS_INFO_THROTTLE(10, "amount of pubs: %zu", realtime_pubs_.size());
  // limit rate of publishing
  for (unsigned i = 0; i < realtime_pubs_.size(); i++) {
    if (publish_rate_ > 0.0 &&
        last_publish_times_[i] + ros::Duration(1.0 / publish_rate_) < time) {
      ROS_INFO_THROTTLE(10, "last_publish_times_: %f",
                        last_publish_times_[i].toSec());
      // try to publish
      if (realtime_pubs_[i]->trylock()) {

        ROS_INFO_THROTTLE(10, "realtime_pubs_ %d", i);
        // we're actually publishing, so increment time
        last_publish_times_[i] =
            last_publish_times_[i] + ros::Duration(1.0 / publish_rate_);

        // populate message
        //        realtime_pubs_[i]->msg_.head.stamp = time;

        ROS_INFO_THROTTLE(10, "header created");
        // TODO(TIM) Set real message!
        march4cpp::PowerDistributionBoard *pBoard =
            pdb_state_[i].getPowerDistributionBoard();
        realtime_pubs_[i]->msg_.low_voltage =
            createPowerNetMessage(pBoard->getLowVoltage());
        realtime_pubs_[i]->msg_.high_voltage =
            createPowerNetMessage(pBoard->getHighVoltage());
        realtime_pubs_[i]->msg_.master_shutdown_requested =
            pBoard->getMasterShutdownRequested();
        realtime_pubs_[i]->msg_.power_distribution_board_current =
            pBoard->getPowerDistributionBoardCurrent();
        ROS_INFO_THROTTLE(10, "netCurrent set");
        realtime_pubs_[i]->unlockAndPublish();
        ROS_INFO_THROTTLE(10, "unlockAndPublish");


      }
    }
  }
}

void MarchPdbStateController::stopping(const ros::Time & /*time*/) {
  ROS_INFO("stopping MarchPdbStateController");
}
} // namespace march_pdb_state_controller

PLUGINLIB_EXPORT_CLASS(march_pdb_state_controller::MarchPdbStateController,
                       controller_interface::ControllerBase)
