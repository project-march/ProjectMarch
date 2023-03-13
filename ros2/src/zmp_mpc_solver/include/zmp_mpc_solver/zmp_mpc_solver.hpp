// standard
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
class ZmpSolver {
public:
    ZmpSolver();
    double m_time_horizon;
    void set_current_state(std::vector<double>);
    int solve_step(std::array<double, NX>&, std::array<double, NU * ZMP_PENDULUM_ODE_N>&);
    int solve_zmp_mpc(std::array<double, NX>&, std::array<double, NU * ZMP_PENDULUM_ODE_N>&);
    std::array<double, NX> get_state();
    std::array<double, NU * ZMP_PENDULUM_ODE_N> get_input_trajectory();

    void set_current_foot(double, double);
    void set_previous_foot(double, double);
    void set_current_com(double, double, double, double);
    void set_current_zmp(double, double);
    void initialize_mpc_params();

private:
    std::array<double, NX> m_x_current;
    std::array<double, NU * ZMP_PENDULUM_ODE_N> m_u_current;
    std::array<double, 2> m_pos_foot_current;
    std::array<double, 2> m_pos_foot_prev;
    std::array<double, 2> m_zmp_current;
    std::array<double, 2> m_com_current;
    std::array<double, 2> m_com_vel_current;

    // Constraints for the ZMP MPC
    int m_number_of_footsteps;

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