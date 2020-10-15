#ifndef _GAZEBO_CONTACT_PLUGIN_HH_
#define _GAZEBO_CONTACT_PLUGIN_HH_

#include <string>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

namespace gazebo
{
/// \brief An example plugin for a contact sensor.
class ContactPlugin : public SensorPlugin
{
  /// \brief Constructor.
public:
  ContactPlugin();

  /// \brief Destructor.
public:
  virtual ~ContactPlugin();

  /// \brief Load the sensor plugin.
  /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
  /// \param[in] _sdf SDF element that describes the plugin.
public:
  virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

  /// \brief Callback that receives the contact sensor's update signal.
private:
  virtual void OnUpdate();

  /// \brief Pointer to the contact sensor
private:
  sensors::ContactSensorPtr parentSensor;

  /// \brief Connection that maintains a link between the contact sensor's
  /// updated signal and the OnUpdate callback.
private:
  event::ConnectionPtr updateConnection;

  std::string name;
  /// \brief A node use for ROS transport
  std::unique_ptr<ros::NodeHandle> ros_node_;

  /// \brief A ROS subscriber
  ros::Publisher ros_pub_;
};
}  // namespace gazebo
#endif