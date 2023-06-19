// standard
#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "zmp_mpc_solver/c_generated_code/main_ZMP_pendulum_ode.cpp"
#include <array>
#include <chrono>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#ifndef ZMP_MPC
#define ZMP_MPC

enum Gait_type { startup, continuous, stopping };

class ZmpSolver {
public:
    ZmpSolver();
    double m_time_horizon;
    bool m_is_weight_shift_done;
    void set_current_state();
    int solve_step();
    int get_current_stance_foot();
    std::array<double, NX> get_state();
    std::array<double, NX * ZMP_PENDULUM_ODE_N>* get_state_trajectory();
    std::array<double, NU * ZMP_PENDULUM_ODE_N> get_input_trajectory();

    void set_current_foot(double, double);
    void update_current_foot();
    void set_previous_foot(double, double);
    void set_current_com(double, double, double, double);
    void set_com_height(double);
    void set_current_zmp(double, double);
    void set_current_stance_foot(int);
    void initialize_mpc_params();
    void set_right_foot_on_gound(bool);
    void set_left_foot_on_gound(bool);
    void set_candidate_footsteps(geometry_msgs::msg::PoseArray::SharedPtr);
    void set_reference_stepsize(std::vector<geometry_msgs::msg::Point>);
    void reset_to_double_stance();
    const std::vector<geometry_msgs::msg::Point>& get_candidate_footsteps() const
    {
        return m_candidate_footsteps;
    }
    double get_com_height();
    void update_current_shooting_node();
    std::vector<double> get_real_time_com_trajectory_x();
    std::vector<double> get_real_time_com_trajectory_y();
    void set_m_current_shooting_node(int);
    std_msgs::msg::Int32 get_m_current_shooting_node();
    bool check_zmp_on_foot();

private:
    int solve_zmp_mpc(std::array<double, NX>&, std::array<double, NU * ZMP_PENDULUM_ODE_N>&);

    std::array<double, NX> m_x_current;
    std::array<double, NX * ZMP_PENDULUM_ODE_N> m_x_trajectory;
    std::array<double, NU * ZMP_PENDULUM_ODE_N> m_u_current;
    std::array<double, 2> m_pos_foot_current;
    std::array<double, 2> m_pos_foot_prev;
    std::array<double, 2> m_zmp_current;
    std::array<double, 2> m_com_current;
    std::array<double, 2> m_com_vel_current;

    std::vector<geometry_msgs::msg::Point> m_candidate_footsteps;
    std::vector<double> m_reference_stepsize_x;
    std::vector<double> m_reference_stepsize_y;
    std::vector<double> m_real_time_com_trajectory_x;
    std::vector<double> m_real_time_com_trajectory_y;

    // Constraints for the ZMP MPC
    int m_number_of_footsteps;

    bool m_right_foot_on_ground;
    bool m_left_foot_on_ground;

    double m_switch;
    int m_current_shooting_node;
    double m_timing_value;

    int m_current_count;
    int m_current_stance_foot;
    int m_previous_stance_foot;
    int m_step_counter;

    double m_admissible_region_x;
    double m_admissible_region_y;
    double m_foot_width_x;
    double m_foot_width_y;
    double m_step_size_x;
    double m_step_size_y;
    double m_com_height;
    const double m_gravity_const;
    double m_first_admissible_region_y;
};

#endif