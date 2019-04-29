// Copyright 2019 Project March.

#ifndef MARCH_STATE_CONTROLLER_MARCH_STATE_CONTROLLER_H
#define MARCH_STATE_CONTROLLER_MARCH_STATE_CONTROLLER_H

#include <controller_interface/controller.h>
#include <march_hardware_interface/march_state_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/Imu.h>
#include <realtime_tools/realtime_publisher.h>
#include <boost/shared_ptr.hpp>

namespace march_state_controller
{
// this controller gets access to the MarchStateInterface
class MarchStateController : public controller_interface::Controller<march_hardware_interface::MarchStateInterface>
{
public:
  MarchStateController()
  {
  }

  virtual bool init(march_hardware_interface::MarchStateInterface* hw, ros::NodeHandle& root_nh,
                    ros::NodeHandle& controller_nh);
  virtual void starting(const ros::Time& time);
  virtual void update(const ros::Time& time, const ros::Duration& /*period*/);
  virtual void stopping(const ros::Time& /*time*/);

private:
  std::vector<march_hardware_interface::MarchStateHandle> sensors_;
  typedef boost::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::Imu> > RtPublisherPtr;
  std::vector<RtPublisherPtr> realtime_pubs_;
  std::vector<ros::Time> last_publish_times_;
  double publish_rate_;
};
}  // namespace march_state_controller

#endif
