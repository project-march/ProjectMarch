// Copyright 2019 Project March.

#include <march_pdb_state_controller/march_pdb_state_controller.h>

namespace march_pdb_state_controller {

void MarchPdbStateController::emergencySwitchCallback(
    const std_msgs::Bool::ConstPtr &msg) {
  ROS_INFO("emergencySwitchCallback %d", msg->data);
  pdb_state_.triggerEmergencySwitch(msg->data);
}
void MarchPdbStateController::masterShutdownAllowedCallback(
    const std_msgs::Bool::ConstPtr &msg) {
  ROS_INFO("masterShutdownAllowedCallback %d", msg->data);
  pdb_state_.setMasterShutdownAllowed(msg->data);
}
void MarchPdbStateController::turnHighVoltageNetOnOrOffCallBack(
    const std_msgs::Int8::ConstPtr &msg) {
  ROS_INFO("turnHighVoltageNetOnOrOffCallBack %d", msg->data);
  int net_number = msg->data;
  // sign indicates on or off
  if (net_number > 0) {
    pdb_state_.turnNetOnOrOff(PowerNetType("high_voltage"), true, net_number);
  } else {
    net_number *= -1;
    pdb_state_.turnNetOnOrOff(PowerNetType("high_voltage"), false, net_number);
  }
}
void MarchPdbStateController::turnLowVoltageNetOnOrOffCallBack(
    const std_msgs::Int8::ConstPtr &msg) {
  ROS_INFO("turnLowVoltageNetOnOrOffCallBack %d", msg->data);
  int net_number = msg->data;
  // sign indicates on or off
  if (net_number > 0) {
    pdb_state_.turnNetOnOrOff(PowerNetType("low_voltage"), true, net_number);
  } else {
    net_number *= -1;
    pdb_state_.turnNetOnOrOff(PowerNetType("low_voltage"), false, net_number);
  }
}

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

  if (pdb_state_names.size() == 1) {
    // sensor handle
    pdb_state_ = (hw->getHandle(pdb_state_names[0]));
    // realtime publisher
    RtPublisherPtr rt_pub(new realtime_tools::RealtimePublisher<
                          march_shared_resources::PowerDistributionBoardState>(
        root_nh, "/march/pdb/" + pdb_state_names[0], 4));
    realtime_pubs_ = rt_pub;
  }

  ROS_INFO("Subscriber to "
           "march/power_distribution_board/emergency_switch_triggered");
  sub_emergency = controller_nh.subscribe(
      "emergency_switch_triggered", 1000,
      &MarchPdbStateController::emergencySwitchCallback, this);

  sub_master_shutdown_allowed = controller_nh.subscribe(
      "shutdown_allowed", 1000,
      &MarchPdbStateController::masterShutdownAllowedCallback, this);

  sub_turn_low_net_on_or_off = controller_nh.subscribe(
      "low_voltage_net/on_or_off", 1000,
      &MarchPdbStateController::turnLowVoltageNetOnOrOffCallBack, this);

  sub_turn_high_net_on_or_off = controller_nh.subscribe(
      "high_voltage_net/on_or_off", 1000,
      &MarchPdbStateController::turnHighVoltageNetOnOrOffCallBack, this);

  return true;
}

void MarchPdbStateController::starting(const ros::Time &time) {
  ROS_INFO("starting MarchPdbStateController");
  // initialize time
  last_publish_times_ = time;
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
  // limit rate of publishing
  if (publish_rate_ > 0.0 &&
      last_publish_times_ + ros::Duration(1.0 / publish_rate_) < time) {
    // try to publish
    if (realtime_pubs_->trylock()) {

      // we're actually publishing, so increment time
      last_publish_times_ =
          last_publish_times_ + ros::Duration(1.0 / publish_rate_);
      march4cpp::PowerDistributionBoard *pBoard =
          pdb_state_.getPowerDistributionBoard();
      realtime_pubs_->msg_.low_voltage =
          createPowerNetMessage(pBoard->getLowVoltage());
      realtime_pubs_->msg_.high_voltage =
          createPowerNetMessage(pBoard->getHighVoltage());
      realtime_pubs_->msg_.master_shutdown_requested =
          pBoard->getMasterShutdownRequested();
      realtime_pubs_->msg_.power_distribution_board_current =
          pBoard->getPowerDistributionBoardCurrent();
      realtime_pubs_->unlockAndPublish();
    }
  }
}

void MarchPdbStateController::stopping(const ros::Time & /*time*/) {
  ROS_INFO("stopping MarchPdbStateController");
}
} // namespace march_pdb_state_controller

PLUGINLIB_EXPORT_CLASS(march_pdb_state_controller::MarchPdbStateController,
                       controller_interface::ControllerBase)
