// Copyright 2019 Project March.
#include "march_pdb_state_controller/march_pdb_state_controller.h"

#include <string>
#include <vector>

#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>

#include <march_hardware/HighVoltage.h>
#include <march_hardware/LowVoltage.h>
#include <march_hardware_interface/march_pdb_state_interface.h>
#include <march_shared_resources/HighVoltageNet.h>
#include <march_shared_resources/LowVoltageNet.h>
#include <march_shared_resources/PowerDistributionBoardState.h>

namespace march_pdb_state_controller
{
MarchPdbStateController::MarchPdbStateController() : last_publish_times_(ros::Time(0)), publish_rate_(0.0)
{
}

bool MarchPdbStateController::init(MarchPdbStateInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  // Get all temperature_sensors from the hardware interface
  const std::vector<std::string>& pdb_state_names = hw->getNames();

  // get publishing period
  if (!controller_nh.getParam("publish_rate", this->publish_rate_))
  {
    ROS_ERROR("Parameter 'publish_rate' not set");
    return false;
  }

  if (pdb_state_names.size() == 1)
  {
    // sensor handle
    this->pdb_state_ = (hw->getHandle(pdb_state_names[0]));
    // realtime publisher
    this->rt_pub_ = std::make_unique<realtime_tools::RealtimePublisher<march_shared_resources::PowerDistributionBoardState>>(
        root_nh, "/march/pdb/" + pdb_state_names[0], 4);
  }

  return true;
}

void MarchPdbStateController::starting(const ros::Time& time)
{
  // initialize time
  this->last_publish_times_ = time;
}

std::vector<march_shared_resources::HighVoltageNet>
MarchPdbStateController::createHighVoltageNetsMessage(march::HighVoltage high_voltage)
{
  std::vector<march_shared_resources::HighVoltageNet> high_voltage_net_msgs;
  for (size_t i = 1; i < 9; i++)
  {
    march_shared_resources::HighVoltageNet msg;
    msg.name = std::to_string(i);
    msg.operational = high_voltage.getNetOperational(i);
    msg.overcurrent_triggered = high_voltage.getOvercurrentTrigger(i);
    high_voltage_net_msgs.push_back(msg);
  }
  return high_voltage_net_msgs;
}

std::vector<march_shared_resources::LowVoltageNet>
MarchPdbStateController::createLowVoltageNetsMessage(march::LowVoltage low_voltage)
{
  std::vector<march_shared_resources::LowVoltageNet> low_voltage_net_msgs;
  for (size_t i = 1; i < 3; i++)
  {
    march_shared_resources::LowVoltageNet msg;
    msg.name = std::to_string(i);
    msg.operational = low_voltage.getNetOperational(i);
    msg.current = low_voltage.getNetCurrent(i);
    low_voltage_net_msgs.push_back(msg);
  }
  return low_voltage_net_msgs;
}

void MarchPdbStateController::update(const ros::Time& time, const ros::Duration& /*period*/)
{
  // limit rate of publishing
  if (this->publish_rate_ > 0.0 && this->last_publish_times_ + ros::Duration(1.0 / this->publish_rate_) < time)
  {
    // try to publish
    if (this->rt_pub_->trylock())
    {
      // we're actually publishing, so increment time
      this->last_publish_times_ = this->last_publish_times_ + ros::Duration(1.0 / this->publish_rate_);
      march::PowerDistributionBoard* pdb = this->pdb_state_.getPowerDistributionBoard();
      this->rt_pub_->msg_.header.stamp = ros::Time::now();
      this->rt_pub_->msg_.low_voltage_nets = createLowVoltageNetsMessage(pdb->getLowVoltage());
      this->rt_pub_->msg_.high_voltage_nets = createHighVoltageNetsMessage(pdb->getHighVoltage());
      this->rt_pub_->msg_.master_shutdown_requested = pdb->getMasterShutdownRequested();
      this->rt_pub_->msg_.power_distribution_board_current = pdb->getPowerDistributionBoardCurrent();
      this->rt_pub_->msg_.high_voltage_enabled = pdb->getHighVoltage().getHighVoltageEnabled();
      this->rt_pub_->unlockAndPublish();
    }
  }
}

void MarchPdbStateController::stopping(const ros::Time& /*time*/)
{
}
}  // namespace march_pdb_state_controller

PLUGINLIB_EXPORT_CLASS(march_pdb_state_controller::MarchPdbStateController, controller_interface::ControllerBase)
