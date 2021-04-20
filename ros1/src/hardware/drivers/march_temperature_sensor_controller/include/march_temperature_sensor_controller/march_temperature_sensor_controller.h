// Copyright 2019 Project March.
#ifndef MARCH_TEMPERATURE_SENSOR_CONTROLLER_MARCH_TEMPERATURE_SENSOR_CONTROLLER_H
#define MARCH_TEMPERATURE_SENSOR_CONTROLLER_MARCH_TEMPERATURE_SENSOR_CONTROLLER_H
#include <vector>

#include <boost/shared_ptr.hpp>
#include <controller_interface/controller.h>
#include <march_hardware_interface/march_temperature_sensor_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/Temperature.h>

namespace march_temperature_sensor_controller {
// this controller gets access to the MarchTemperatureSensorInterface
class MarchTemperatureSensorController
    : public controller_interface::Controller<MarchTemperatureSensorInterface> {
public:
    MarchTemperatureSensorController() = default;

    bool init(MarchTemperatureSensorInterface* hw, ros::NodeHandle& root_nh,
        ros::NodeHandle& controller_nh) override;
    void starting(const ros::Time& time) override;
    void update(
        const ros::Time& time, const ros::Duration& /*period*/) override;
    void stopping(const ros::Time& /*time*/) override;

private:
    std::vector<MarchTemperatureSensorHandle> temperature_sensors_;
    typedef boost::shared_ptr<
        realtime_tools::RealtimePublisher<sensor_msgs::Temperature>>
        RtPublisherPtr;
    std::vector<RtPublisherPtr> realtime_pubs_;
    std::vector<ros::Time> last_publish_times_;
    double publish_rate_ {};
};
} // namespace march_temperature_sensor_controller

#endif // MARCH_TEMPERATURE_SENSOR_CONTROLLER_MARCH_TEMPERATURE_SENSOR_CONTROLLER_H
