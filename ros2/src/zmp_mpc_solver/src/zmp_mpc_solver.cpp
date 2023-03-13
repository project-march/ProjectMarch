// standard
#include "zmp_mpc_solver/zmp_mpc_solver.hpp"

ZmpSolver::ZmpSolver()
    : m_x_current({})
    , m_time_horizon(3.0)
    , m_gravity_const(9.81)
{
}

int ZmpSolver::solve_step(std::array<double, NX>& x_cur, std::array<double, NU * ZMP_PENDULUM_ODE_N>& u_cur)
{
    return solve_zmp_mpc(x_cur, u_cur);
    // return 1;
}

std::array<double, NX> ZmpSolver::get_state()
{
    return m_x_current;
}

std::array<double, NU * ZMP_PENDULUM_ODE_N> ZmpSolver::get_input_trajectory()
{
    return m_u_current;
}

void ZmpSolver::set_current_state(std::vector<double> new_state)
{
    std::copy(new_state.begin(), new_state.end(), m_x_current.begin());
}

void ZmpSolver::set_current_foot(double x, double y)
{
    m_pos_foot_current[0] = x;
    m_pos_foot_current[1] = y;
}

void ZmpSolver::set_previous_foot(double x, double y)
{
    m_pos_foot_prev[0] = x;
    m_pos_foot_prev[1] = y;
}

void ZmpSolver::set_current_com(double x, double y, double dx, double dy)
{
    m_com_current[0] = x;
    m_com_current[1] = y;

    m_com_vel_current[0] = x;
    m_com_vel_current[1] = y;
}

void ZmpSolver::set_current_zmp(double x, double y)
{
    m_zmp_current[0] = x;
    m_zmp_current[1] = y;
}

void ZmpSolver::initialize_mpc_params()
{
    // Later, change this to read from a yaml
    m_admissible_region_x = 0.42;
    m_admissible_region_y = 0.10;
    m_foot_width_x = 0.10;
    m_foot_width_y = 0.08;
    m_step_size_x = 0.2;
    m_step_size_y = 0.2;

    m_com_height = 0.6; // Load this from the com position
    m_first_admissible_region_y = 0.01;
}