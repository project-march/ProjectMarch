#ifndef CONTACT_CONTACT_PLUGIN_H
#define CONTACT_CONTACT_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <string>

namespace gazebo {
/// \brief An example plugin for a contact sensor.
class ContactPlugin : public SensorPlugin {

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
public:
    void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

    /// \brief Callback that receives the contact sensor's update signal.
private:
    void OnUpdate();

    /// \brief Pointer to the contact sensor
    sensors::ContactSensorPtr parentSensor;

    /// \brief Connection that maintains a link between the contact sensor's
    /// updated signal and the OnUpdate callback.
    event::ConnectionPtr updateConnection;

    /// \brief The name of the contact sensor
    std::string name;
    /// \brief A node use for ROS transport
    std::unique_ptr<ros::NodeHandle> ros_node_;

    /// \brief A ROS subscriber
    ros::Publisher ros_pub_;
};
} // namespace gazebo
#endif