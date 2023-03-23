#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "joint_estimator.hpp"
#include "march_shared_msgs/msg/robot_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <unordered_map>

#ifndef COP_ESTIMATOR
#define COP_ESTIMATOR

struct CenterOfPressure {
    std::string name;
    geometry_msgs::msg::PointStamped position;
    double pressure;

    /**
     * This function is in place for comparing the struct.
     * This is needed for testing purposes.
     * @param a
     * @param b
     * @return
     */
    friend bool operator==(CenterOfPressure a, CenterOfPressure b)
    {
        return (a.position.point.x == b.position.point.x && a.position.point.y == b.position.point.y
            && a.position.point.z == b.position.point.z && a.pressure == b.pressure);
    }
};

struct PressureSensor {
    std::string name;
    CenterOfPressure centre_of_pressure;

    void update_pressure(double pressure)
    {
        centre_of_pressure.pressure = pressure;
    }
};

class StateEstimator;

class CopEstimator {
public:
    CopEstimator(std::vector<PressureSensor> sensors);
    void set_cop_state(std::vector<PressureSensor> sensors);
    std::vector<PressureSensor> get_sensors();
    void update_sensor_pressures(std::map<std::string, double> pressure_values_map);
    void update_individual_pressure_sensor(std::string name, double pressure);
    CenterOfPressure get_cop_state();

private:
    std::vector<PressureSensor> m_sensors;
    CenterOfPressure m_center_of_pressure;
};

#endif