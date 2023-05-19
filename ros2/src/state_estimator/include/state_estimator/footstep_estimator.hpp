#include "cop_estimator.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
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
#include <vector>

#ifndef FOOTSTEP_ESTIMATOR
#define FOOTSTEP_ESTIMATOR

struct Foot {
    geometry_msgs::msg::PointStamped position;
    double width;
    double height;
    double threshold;
    bool on_ground;
    // The prefix can be L or R
    void set_on_ground(const std::vector<PressureSensor*>* sensors, const char* prefix)
    {
        double measured_foot_pressure = 0.0;
        // look for the right pressure sensors specific to the foot
        for (auto i : *sensors) {
            if (i->name[0] == *prefix) {
                measured_foot_pressure += i->pressure;
            }
        }
        // we have 8 sensors, so we divide by 1

        on_ground = (measured_foot_pressure / 8 >= threshold);
    };
};

class FootstepEstimator {
public:
    FootstepEstimator();
    geometry_msgs::msg::Pose get_foot_position(const char*);
    void set_foot_size(double, double, const char*);
    void update_feet(const std::vector<PressureSensor*>*);
    bool get_foot_on_ground(const char*);
    void set_threshold(double);
    Foot* get_foot(const char*);
    // IMU& get_imu();

private:
    Foot foot_left;
    Foot foot_right;
    double m_threshold;
};

#endif