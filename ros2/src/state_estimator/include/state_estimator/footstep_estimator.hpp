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
    double foot_threshold = 0.0;
    double toes_middle_threshold = 2.74;
    double heel_middle_threshold = 2.55;
    double derivative_treshold = -0.1;
    double previous_foot_pressure;
    std::vector<double> previous_foot_pressures;
    bool weight_on_foot;
    bool impact_ground;
    // The prefix can be L or R
    void set_on_ground(const std::vector<PressureSensor*>* sensors, const char* prefix)
    {
        double measured_foot_pressure = 0.0;
        // look for the right pressure sensors specific to the foot
        std::vector<std::string> toes = { "toes", "hallux" };
        std::vector<std::string> middle = { "met1", "met3", "met5", "arch" };
        std::vector<std::string> heel = { "heel_right", "heel_left" };
        double measured_toes_middle_pressure = 0.0;
        double measured_heel_middle_pressure = 0.0;
        double delta_pressure;
        double measured_foot_pressure_derivative = 0.0;
        int foot_sensor_amount = 0;
        int toes_middle_sensor_amount = 0;
        int heel_middle_sensor_amount = 0;
        for (auto i : *sensors) {
            if (i->name[0] == *prefix) {
                measured_foot_pressure += i->pressure;
                // We only take substring 2 to end because index 0 and 1 are the prefix with underscore.
                if (std::find(toes.begin(), toes.end(), i->name.substr(2, i->name.size())) != toes.end()) {
                    measured_toes_middle_pressure += i->pressure;
                    toes_middle_sensor_amount++;
                }
                if (std::find(middle.begin(), middle.end(), i->name.substr(2, i->name.size())) != middle.end()) {
                    measured_toes_middle_pressure += i->pressure;
                    measured_heel_middle_pressure += i->pressure;
                    toes_middle_sensor_amount++;
                    heel_middle_sensor_amount++;
                }
                if (std::find(heel.begin(), heel.end(), i->name.substr(2, i->name.size())) != heel.end()) {
                    measured_heel_middle_pressure += i->pressure;
                    heel_middle_sensor_amount ++;
                }    
                if (previous_foot_pressure != 0.0) {
                    delta_pressure = i->pressure - previous_foot_pressures.at(foot_sensor_amount);
                    measured_foot_pressure_derivative += delta_pressure;
                }
                foot_sensor_amount++;
            }
        }
        // we have 8 sensors, so we divide by 1
        foot_threshold = 2.7;
        derivative_treshold = -0.1;
        toes_middle_threshold = 2.74;
        heel_middle_threshold = 2.55;

        if (measured_foot_pressure <= 0.00001 || measured_foot_pressure > 10000) {
            weight_on_foot = false;
            return;
        }

        weight_on_foot = ((measured_foot_pressure / foot_sensor_amount) <= foot_threshold ||
                    (measured_heel_middle_pressure / heel_middle_sensor_amount) <= heel_middle_sensor_amount ||
                    (measured_toes_middle_pressure / toes_middle_sensor_amount) <= toes_middle_threshold);
        impact_ground = (measured_foot_pressure_derivative / foot_sensor_amount <= derivative_treshold);
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
    void set_footstep_positions(geometry_msgs::msg::Vector3 right_foot_vec, geometry_msgs::msg::Vector3 left_foot_vec);

    // IMU& get_imu();

private:
    Foot foot_left;
    Foot foot_right;
    double m_threshold;
};

#endif