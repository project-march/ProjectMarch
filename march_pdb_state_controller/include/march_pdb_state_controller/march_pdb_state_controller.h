// Copyright 2019 Project March.

#ifndef MARCH_PDB_STATE_CONTROLLER_MARCH_PDB_STATE_CONTROLLER_H
#define MARCH_PDB_STATE_CONTROLLER_MARCH_PDB_STATE_CONTROLLER_H
#include <vector>

#include <controller_interface/controller.h>
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
class MarchPdbStateController : public controller_interface::Controller<MarchPdbStateInterface>
{
public:
  MarchPdbStateController();

  bool init(MarchPdbStateInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& /*period*/) override;
  void stopping(const ros::Time& /*time*/) override;

private:
  static std::vector<march_shared_resources::HighVoltageNet>
  createHighVoltageNetsMessage(march::HighVoltage high_voltage);
  static std::vector<march_shared_resources::LowVoltageNet> createLowVoltageNetsMessage(march::LowVoltage low_voltage);

  MarchPdbStateHandle pdb_state_;
  std::unique_ptr<realtime_tools::RealtimePublisher<march_shared_resources::PowerDistributionBoardState>> rt_pub_;
  ros::Time last_publish_times_;
  double publish_rate_;
};
}  // namespace march_pdb_state_controller

#endif  // MARCH_PDB_STATE_CONTROLLER_MARCH_PDB_STATE_CONTROLLER_H
