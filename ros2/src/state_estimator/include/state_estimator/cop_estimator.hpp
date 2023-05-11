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

struct PressureSensor {
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
    friend bool operator==(PressureSensor a, PressureSensor b)
    {
        return (a.position.point.x == b.position.point.x && a.position.point.y == b.position.point.y
            && a.position.point.z == b.position.point.z && a.pressure == b.pressure && a.name == b.name);
    }
};

class StateEstimator;

class CopEstimator {
public:
    CopEstimator(std::vector<PressureSensor*> sensors);

    void update_pressure_sensors(std::map<std::string, double> pressure_values_map);
    void update_individual_pressure_sensor(std::string name, double pressure);

    std::vector<PressureSensor*>* get_sensors();

    void set_cop(
        std::vector<PressureSensor*> sensors, std::array<geometry_msgs::msg::TransformStamped, 2> reference_frames);
    geometry_msgs::msg::PointStamped get_cop();

private:
    std::vector<PressureSensor*> m_sensors;
    geometry_msgs::msg::PointStamped m_center_of_pressure;
};

#endif