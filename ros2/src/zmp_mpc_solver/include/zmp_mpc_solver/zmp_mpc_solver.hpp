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
    std::array<double, NX> get_state();
    std::array<double, NU * ZMP_PENDULUM_ODE_N> get_input_trajectory();

private:
    std::array<double, NX> m_x_current;
    std::array<double, NU * ZMP_PENDULUM_ODE_N> m_u_current;
};

#endif