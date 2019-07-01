// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_STATE_CONTROLLER_H
#define MARCH_HARDWARE_STATE_CONTROLLER_H

#include <controller_interface/controller.h>
#include <march_hardware_interface/march_hardware_state_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <boost/shared_ptr.hpp>
#include <std_msgs/Empty.h>

namespace march_hardware_state_controller
{
// this controller gets access to the MarchHardwareStateInterface
class MarchHardwareStateController
    : public controller_interface::Controller<march_hardware_interface::MarchHardwareStateInterface>
{
public:
  MarchHardwareStateController()
  {
  }

  virtual bool init(march_hardware_interface::MarchHardwareStateInterface* hw, ros::NodeHandle& root_nh,
                    ros::NodeHandle& controller_nh);
  virtual void starting(const ros::Time& time);
  virtual void update(const ros::Time& time, const ros::Duration& /*period*/);
  virtual void stopping(const ros::Time& /*time*/);

private:
  std::vector<march_hardware_interface::MarchHardwareStateHandle> hardware_states_;
  typedef boost::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Empty> > RtPublisherPtr;
  std::vector<RtPublisherPtr> realtime_pubs_;
  std::vector<ros::Time> last_publish_times_;
  double publish_rate_;
};
}  // namespace march_hardware_state_controller

#endif  // MARCH_HARDWARE_STATE_CONTROLLER_H
