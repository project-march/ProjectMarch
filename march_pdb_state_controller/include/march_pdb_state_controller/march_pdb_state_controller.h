// Copyright 2019 Project March.

#ifndef MARCH_PDB_STATE_CONTROLLER_MARCH_PDB_STATE_CONTROLLER_H
#define MARCH_PDB_STATE_CONTROLLER_MARCH_PDB_STATE_CONTROLLER_H
#include <vector>

#include <boost/lexical_cast.hpp>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_srvs/SetBool.h>
#include <string>

#include <boost/shared_ptr.hpp>
#include <controller_interface/controller.h>
#include <march_hardware_interface/march_pdb_state_interface.h>
#include <march_shared_resources/PowerDistributionBoardState.h>
#include <march_shared_resources/HighVoltageNet.h>
#include <march_shared_resources/LowVoltageNet.h>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_publisher.h>

namespace march_pdb_state_controller
{
class MarchPdbStateController : public controller_interface::Controller<MarchPdbStateInterface>
{
public:
  MarchPdbStateController()
  {
  }

  virtual bool init(MarchPdbStateInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  virtual void starting(const ros::Time& time);
  virtual void update(const ros::Time& time, const ros::Duration& /*period*/);
  virtual void stopping(const ros::Time& /*time*/);

private:
  MarchPdbStateHandle pdb_state_;
  typedef boost::shared_ptr<realtime_tools::RealtimePublisher<march_shared_resources::PowerDistributionBoardState> >
      RtPublisherPtr;
  RtPublisherPtr realtime_pubs_;
  ros::Time last_publish_times_;
  double publish_rate_;

  ros::ServiceServer enable_disable_high_voltage_service;
  ros::Subscriber sub_master_shutdown_allowed;
  ros::Subscriber sub_turn_low_net_on_or_off;
  ros::Subscriber sub_turn_high_net_on_or_off;

  static std::vector<march_shared_resources::HighVoltageNet>
  createHighVoltageNetsMessage(march::HighVoltage high_voltage);
  static std::vector<march_shared_resources::LowVoltageNet> createLowVoltageNetsMessage(march::LowVoltage low_voltage);
  bool serviceDisableEnableHighVoltage(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  void masterShutdownAllowedCallback(const std_msgs::Bool::ConstPtr& msg);
  void turnHighVoltageNetOnOrOffCallBack(const std_msgs::Int8::ConstPtr& msg);
  void turnLowVoltageNetOnOrOffCallBack(const std_msgs::Int8::ConstPtr& msg);
};
}  // namespace march_pdb_state_controller

#endif  // MARCH_PDB_STATE_CONTROLLER_MARCH_PDB_STATE_CONTROLLER_H
