// Copyright 2019 Project March.

#ifndef MARCH_PDB_STATE_CONTROLLER_H
#define MARCH_PDB_STATE_CONTROLLER_H

#include <controller_interface/controller.h>
#include <march_hardware_interface/march_pdb_state_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <march_shared_resources/PowerDistributionBoardState.h>
#include <realtime_tools/realtime_publisher.h>
#include <boost/shared_ptr.hpp>

namespace march_pdb_state_controller
{
class MarchPdbStateController
    : public controller_interface::Controller<march_hardware_interface::MarchPdbStateInterface>
{
public:
  MarchPdbStateController()
  {
  }

  virtual bool init(march_hardware_interface::MarchPdbStateInterface* hw, ros::NodeHandle& root_nh,
                    ros::NodeHandle& controller_nh);
  virtual void starting(const ros::Time& time);
  virtual void update(const ros::Time& time, const ros::Duration& /*period*/);
  virtual void stopping(const ros::Time& /*time*/);

private:
  std::vector<march_hardware_interface::MarchPdbStateHandle> pdb_state_;
  typedef boost::shared_ptr<realtime_tools::RealtimePublisher<march_shared_resources::PowerDistributionBoardState> > RtPublisherPtr;
  std::vector<RtPublisherPtr> realtime_pubs_;
  std::vector<ros::Time> last_publish_times_;
  double publish_rate_;
};
}  // namespace march_pdb_state_controller

#endif  // MARCH_PDB_STATE_CONTROLLER_H
