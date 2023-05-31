//
// Created by Jack Zeng on 30-05-2023.
//

#ifndef STATE_ESTIMATOR_MOCK_HPP
#define STATE_ESTIMATOR_MOCK_HPP
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "march_shared_msgs/msg/center_of_mass.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"

class StateEstimatorMock {
public:
    StateEstimatorMock();
    int get_current_shooting_node();
    void set_current_shooting_node(int);
    double gaussian_dist(int, double, double);
    march_shared_msgs::msg::CenterOfMass get_current_com();
    geometry_msgs::msg::PointStamped get_current_zmp();
    geometry_msgs::msg::PoseArray get_previous_foot();
    std_msgs::msg::Int32 get_current_stance_foot();
    std_msgs::msg::Bool get_right_foot_ground();
    std_msgs::msg::Bool get_left_foot_ground();

private:
    int m_current_shooting_node;
    int m_current_stance_foot;
    int m_shooting_nodes_per_step;

    double m_step_duration;
    double m_center_of_mass_height;
};

#endif // STATE_ESTIMATOR_MOCK_HPP