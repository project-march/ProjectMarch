// standard
#include "zmp_mpc_solver/zmp_mpc_solver.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

ZmpSolver::ZmpSolver()
    : m_x_current({})
    , m_time_horizon(3.0)
{
}

int ZmpSolver::solve_step(std::array<double, NX>& x_cur, std::array<double, NU * ZMP_PENDULUM_ODE_N>& u_cur)
{
    return solve_zmp_mpc(x_cur, u_cur);
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