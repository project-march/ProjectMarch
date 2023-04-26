// standard
#include "zmp_mpc_solver/c_generated_code/main_ZMP_pendulum_ode.cpp"
#include "geometry_msgs/msg/pose_array.hpp"
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
    void set_current_state();
    int solve_step();
    std::array<double, NX> get_state();
    std::array<double, NX * ZMP_PENDULUM_ODE_N>* get_state_trajectory();
    std::array<double, NU * ZMP_PENDULUM_ODE_N> get_input_trajectory();

    void set_current_foot(double, double);
    void set_previous_foot(double, double);
    void set_current_com(double, double, double, double);
    void set_com_height(double);
    void set_current_zmp(double, double);
    void set_current_stance_foot(int);
    void initialize_mpc_params();
    void set_candidate_footsteps(geometry_msgs::msg::PoseArray::SharedPtr);
    void set_reference_stepsize(geometry_msgs::msg::Point);
    double get_com_height();

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

    // Constraints for the ZMP MPC
    int m_number_of_footsteps;

    double m_switch;
    int m_current_shooting_node;
    double m_timing_value;

    int m_current_stance_foot;

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