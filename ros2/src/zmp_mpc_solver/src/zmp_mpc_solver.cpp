// standard
#include "zmp_mpc_solver/zmp_mpc_solver.hpp"

ZmpSolver::ZmpSolver()
    : m_x_current({ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 })
    , m_u_current({ 0, 0, 0, 0, 0, 0, 0 })
    , m_time_horizon(3.0)
    , m_gravity_const(9.81)
{
    initialize_mpc_params();

    solve_step(m_x_current, m_u_current);
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

inline int ZmpSolver::solve_zmp_mpc(
    std::array<double, NX>& x_init_input, std::array<double, NU * ZMP_PENDULUM_ODE_N>& u_current)
{

    ZMP_pendulum_ode_solver_capsule* acados_ocp_capsule = ZMP_pendulum_ode_acados_create_capsule();
    // there is an opportunity to change the number of shooting intervals in C without new code generation
    int N = ZMP_PENDULUM_ODE_N;
    // allocate the array and fill it accordingly
    double* new_time_steps = NULL;
    int status = ZMP_pendulum_ode_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);

    if (status) {
        printf("ZMP_pendulum_ode_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    ocp_nlp_config* nlp_config = ZMP_pendulum_ode_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims* nlp_dims = ZMP_pendulum_ode_acados_get_nlp_dims(acados_ocp_capsule);
    ocp_nlp_in* nlp_in = ZMP_pendulum_ode_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out* nlp_out = ZMP_pendulum_ode_acados_get_nlp_out(acados_ocp_capsule);
    ocp_nlp_solver* nlp_solver = ZMP_pendulum_ode_acados_get_nlp_solver(acados_ocp_capsule);
    void* nlp_opts = ZMP_pendulum_ode_acados_get_nlp_opts(acados_ocp_capsule);

    // initial condition
    int idxbx0[NBX0];
    idxbx0[0] = 0;
    idxbx0[1] = 1;
    idxbx0[2] = 2;
    idxbx0[3] = 3;
    idxbx0[4] = 4;
    idxbx0[5] = 5;
    idxbx0[6] = 6;
    idxbx0[7] = 7;
    idxbx0[8] = 8;
    idxbx0[9] = 9;
    idxbx0[10] = 10;
    idxbx0[11] = 11;

    double lbx0[NBX0];
    double ubx0[NBX0];
    lbx0[0] = x_init_input[0];
    ubx0[0] = x_init_input[0];
    lbx0[1] = x_init_input[1];
    ubx0[1] = x_init_input[1];
    lbx0[2] = x_init_input[2];
    ubx0[2] = x_init_input[2];
    lbx0[3] = x_init_input[3];
    ubx0[3] = x_init_input[3];
    lbx0[4] = x_init_input[4];
    ubx0[4] = x_init_input[4];
    lbx0[5] = x_init_input[5];
    ubx0[5] = x_init_input[5];
    lbx0[6] = x_init_input[6];
    ubx0[6] = x_init_input[6];
    lbx0[7] = x_init_input[7];
    ubx0[7] = x_init_input[7];
    lbx0[8] = x_init_input[8];
    ubx0[8] = x_init_input[8];
    lbx0[9] = x_init_input[9];
    ubx0[9] = x_init_input[9];
    lbx0[10] = x_init_input[10];
    ubx0[10] = x_init_input[10];
    lbx0[11] = x_init_input[11];
    ubx0[11] = x_init_input[11];

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

    // initialization for state values
    double x_init[NX];
    x_init[0] = x_init_input[0];
    x_init[1] = x_init_input[1];
    x_init[2] = x_init_input[2];
    x_init[3] = x_init_input[3];
    x_init[4] = x_init_input[4];
    x_init[5] = x_init_input[5];
    x_init[6] = x_init_input[6];
    x_init[7] = x_init_input[7];
    x_init[8] = x_init_input[8];
    x_init[9] = x_init_input[9];
    x_init[10] = x_init_input[10];
    x_init[11] = x_init_input[11];

    // initial value for control input
    double u0[NU];
    u0[0] = 0.0;
    u0[1] = 0.0;
    u0[2] = 0.0;
    u0[3] = 0.0;
    // set parameters
    // WE NEED TO SET OUR PARAMETERS HERE
    double p[NP];
    p[0] = 0;
    p[1] = 0;
    p[2] = 0;
    p[3] = 0;
    p[4] = 0;
    p[5] = 0;
    p[6] = 0;

    for (int ii = 0; ii <= N; ii++) {
        ZMP_pendulum_ode_acados_update_params(acados_ocp_capsule, ii, p, NP);
    }

    // prepare evaluation
    int NTIMINGS = 1;
    double min_time = 1e12;
    double kkt_norm_inf;
    double elapsed_time;
    int sqp_iter;

    double xtraj[NX * (N + 1)];
    double utraj[NU * N];

    // solve ocp in loop
    int rti_phase = 0;

    for (int ii = 0; ii < NTIMINGS; ii++) {
        // initialize solution
        for (int i = 0; i < N; i++) {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_init);
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
        }
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x_init);
        ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_phase", &rti_phase);
        status = ZMP_pendulum_ode_acados_solve(acados_ocp_capsule);
        ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
        min_time = MIN(elapsed_time, min_time);
    }

    /* print solution and statistics */
    for (int ii = 0; ii <= nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &xtraj[ii * NX]);
    for (int ii = 0; ii < nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &utraj[ii * NU]);

    printf("\n--- xtraj ---\n");
    d_print_exp_tran_mat(NX, N + 1, xtraj, NX);
    printf("\n--- utraj ---\n");
    d_print_exp_tran_mat(NU, N, utraj, NU);
    // ocp_nlp_out_print(nlp_solver->dims, nlp_out);

    printf("\nsolved ocp %d times, solution printed above\n\n", NTIMINGS);

    if (status == ACADOS_SUCCESS) {
        printf("ZMP_pendulum_ode_acados_solve(): SUCCESS!\n");
    } else {
        printf("ZMP_pendulum_ode_acados_solve() failed with status %d.\n", status);
    }

    // here, we copy our array into the std::array
    for (int ii = 0; ii < nlp_dims->N; ii++) {
        u_current[ii] = utraj[ii];
    }

    for (int ii = 0; ii < NX; ii++) {
        x_init_input[ii] = xtraj[NX + ii];
    }

    // get solution
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter);

    ZMP_pendulum_ode_acados_print_stats(acados_ocp_capsule);

    printf("\nSolver info:\n");
    printf(" SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n", sqp_iter, NTIMINGS, min_time * 1000,
        kkt_norm_inf);

    // free solver
    status = ZMP_pendulum_ode_acados_free(acados_ocp_capsule);
    if (status) {
        printf("ZMP_pendulum_ode_acados_free() returned status %d. \n", status);
    }
    // free solver capsule
    status = ZMP_pendulum_ode_acados_free_capsule(acados_ocp_capsule);
    if (status) {
        printf("ZMP_pendulum_ode_acados_free_capsule() returned status %d. \n", status);
    }

    return status;
}