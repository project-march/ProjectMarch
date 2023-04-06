// standard
#include "zmp_mpc_solver/zmp_mpc_solver.hpp"

ZmpSolver::ZmpSolver()
    : m_x_current()
    , m_u_current()
    , m_time_horizon(10.0)
    , m_gravity_const(9.81)
    , m_switch(0)
    , m_current_shooting_node(0)
    , m_timing_value(0)
{
    initialize_mpc_params();
    m_x_current.fill(0);
    m_x_trajectory.fill(0);
    m_u_current.fill(0);

    set_current_com(0.0, 0.06, 0.0, -0.1);
    set_current_zmp(0.0, 0.06);
    set_current_foot(0.0, 0.1);
    set_previous_foot(0.0, 0.1);
    set_current_state();
    // m_x_current = {0.0, 0.0, 0.0, 0.06, -0.1, 0.06, 0.0, 0.0, 0.1, 0.1, 0.0, 0.0};
    solve_step();
}

const double ZmpSolver::get_com_height()
{
    return m_com_height;
}

int ZmpSolver::solve_step()
{
    return solve_zmp_mpc(m_x_current, m_u_current);
    // return 1;
}

std::array<double, NX> ZmpSolver::get_state()
{
    return m_x_current;
}

std::array<double, NX * ZMP_PENDULUM_ODE_N>* ZmpSolver::get_state_trajectory()
{
    return &m_x_trajectory;
}

std::array<double, NU * ZMP_PENDULUM_ODE_N> ZmpSolver::get_input_trajectory()
{
    return m_u_current;
}

void ZmpSolver::set_current_state()
{
    // This is of course MPC dependent
    m_x_current[0] = m_com_current[0];
    m_x_current[1] = m_com_vel_current[0];

    m_x_current[2] = m_zmp_current[0];

    m_x_current[3] = m_com_current[1];
    m_x_current[4] = m_com_vel_current[1];

    m_x_current[5] = m_zmp_current[1];

    m_x_current[6] = m_pos_foot_current[0];
    m_x_current[7] = m_pos_foot_prev[0];

    m_x_current[8] = m_pos_foot_current[1];
    m_x_current[9] = m_pos_foot_prev[1];

    m_x_current[10] = 0;
    m_x_current[11] = 0;
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

    m_com_vel_current[0] = dx;
    m_com_vel_current[1] = dy;
}

void ZmpSolver::set_current_zmp(double x, double y)
{
    m_zmp_current[0] = x;
    m_zmp_current[1] = y;
}

void ZmpSolver::initialize_mpc_params()
{
    // Later, change this to read from a yaml
    m_admissible_region_x = 0.62;
    m_admissible_region_y = 0.10;
    m_foot_width_x = 0.10;
    m_foot_width_y = 0.1;
    m_step_size_x = 0.2;
    m_step_size_y = 0.2;

    m_com_height = 0.6; // Load this from the com position
    m_first_admissible_region_y = 0.01;

    m_switch = 1.0;
    m_current_shooting_node = 0;
    m_timing_value = 0;

    m_number_of_footsteps = 10;
}

inline int ZmpSolver::solve_zmp_mpc(
    std::array<double, NX>& x_init_input, std::array<double, NU * ZMP_PENDULUM_ODE_N>& u_current)
{
    ZMP_pendulum_ode_solver_capsule* acados_ocp_capsule = ZMP_pendulum_ode_acados_create_capsule();
    // there is an opportunity to change the number of shooting intervals in C without new code generation
    int N = ZMP_PENDULUM_ODE_N;
    // allocate the array and fill it accordingly
    double* new_time_steps = NULL;
    printf("Shooting nodes: %i\n", N);
    // First, we do some checks so the mpc works
    if ((N - 1) % m_number_of_footsteps != 0) {
        return 5;
    }

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

    // We define our constraints here
    // lh: Lower path constraints
    double lh[4];
    // rh: Upper path constraints
    double uh[4];

    // // lhe: TERMINAL Lower path constraints
    // double lhe[2];
    // lhe[0] = -m_foot_width_x;
    // lhe[1] = -m_foot_width_y;
    // // rhe: TERMINAL Upper path constraints
    // double uhe[2];
    // uhe[0] = m_foot_width_x;
    // uhe[1] = m_foot_width_y;
    // ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lh_e", lhe);
    // ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "uh_e", uhe);

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
    // p[0] : Planned footstep width;
    // p[1] : Planned footstep height;
    // p[2] : Switch variable(0 at all times, 1 when we want to take a step);
    // p[3] : moving foot constraint timing variable(0-1);
    // p[4] : Periodic tail constraint;
    // p[5] : RK4 constraint (always 0);
    // p[6] : RK4 constraint (always 0);
    p[0] = 0;
    p[1] = 0;
    p[2] = 0;
    p[3] = 0;
    p[4] = 0;
    printf("%f \n", (m_time_horizon) / N);
    printf("%f \n", (10.0 / 61));
    double dt = 0.0 + (m_time_horizon) / N;
    // If the footstep is the left foot or the right foot(left is -1, right is 1)
    double count = -1;
    m_timing_value = 0.0;
    m_switch = 1.0 / dt;
    if ((m_current_shooting_node != 0) && (m_current_shooting_node < ((N - 1) / m_number_of_footsteps))) {
        m_timing_value = (m_current_shooting_node - 1) / ((N - 1) / m_number_of_footsteps);
    }

    // ii is defined as the current stage
    for (int ii = 0; ii <= N; ii++) {
        if (((ii + m_current_shooting_node) % ((N - 1) / m_number_of_footsteps) == 0)
            && (ii + m_current_shooting_node != 0)) {
            count = -count;
            m_timing_value = 1;

            p[0] = m_step_size_x / dt;
            p[1] = -count * m_step_size_y / dt;
            p[2] = m_switch;
            p[3] = m_timing_value;
            p[4] = 0;

            ZMP_pendulum_ode_acados_update_params(acados_ocp_capsule, ii, p, NP);
            m_timing_value = -1.0 / (((N - 1) - m_number_of_footsteps) / (m_number_of_footsteps));

        } else if (ii + m_current_shooting_node == 0) {
            // STARTING SHOOTING NODE
            count = -count;
            m_timing_value = 1;

            p[0] = 0;
            p[1] = -count * m_step_size_y / dt;
            p[2] = m_switch;
            p[3] = m_timing_value;
            p[4] = 0;

            ZMP_pendulum_ode_acados_update_params(acados_ocp_capsule, ii, p, NP);
            m_timing_value = -1.0 / (((N - 1) - m_number_of_footsteps) / (m_number_of_footsteps));

        } else if (ii + m_current_shooting_node
            == (2 * (m_current_shooting_node + (N - 1) / m_number_of_footsteps) - 1)) {
            m_timing_value += 1.0 / (((N - 1) - m_number_of_footsteps) / (m_number_of_footsteps));
            p[0] = 0;
            p[1] = 0;
            p[2] = 0;
            p[3] = m_timing_value;
            p[4] = 1;

            ZMP_pendulum_ode_acados_update_params(acados_ocp_capsule, ii, p, NP);
        } else if (ii + m_current_shooting_node == 2 * ((N - 1) / m_number_of_footsteps)) {
            count = -count;
            m_timing_value = 1;
            p[0] = m_step_size_x / dt / 2;
            p[1] = -count * m_step_size_y / dt;
            p[2] = m_switch;
            p[3] = m_timing_value;
            p[4] = 0;

            ZMP_pendulum_ode_acados_update_params(acados_ocp_capsule, ii, p, NP);

        } else if ((ii + m_current_shooting_node) % ((N - 1) / m_number_of_footsteps) == 0
            && (ii + m_current_shooting_node) >= (2 * ((N - 1) / m_number_of_footsteps))) {
            count = -count;
            m_timing_value = 1;
            p[0] = 0;
            p[1] = -count * m_step_size_y;
            p[2] = m_switch;
            p[3] = m_timing_value;
            p[4] = 0;

            ZMP_pendulum_ode_acados_update_params(acados_ocp_capsule, ii, p, NP);
            m_timing_value = -1 / (((N)-m_number_of_footsteps) / m_number_of_footsteps);
        } else {
            // Standard weight shift
            m_timing_value += 1.0 / (((N - 1) - m_number_of_footsteps) / (m_number_of_footsteps));
            p[0] = 0;
            p[1] = 0;
            p[2] = 0;
            p[3] = m_timing_value;
            p[4] = 0;

            ZMP_pendulum_ode_acados_update_params(acados_ocp_capsule, ii, p, NP);
        }

        // LOWER_CONSTRAINT
        lh[0] = -0.5 * m_admissible_region_x;
        lh[1] = -count * m_step_size_y - 0.5 * m_admissible_region_y;
        lh[2] = -m_foot_width_x / 2;
        lh[3] = -m_foot_width_y / 2;
        // UPPER_CONSTRAINT
        uh[0] = 0.5 * m_admissible_region_x;
        uh[1] = -count * m_step_size_y + 0.5 * m_admissible_region_y;
        uh[2] = m_foot_width_x / 2;
        uh[3] = m_foot_width_y / 2;
        //
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "lh", lh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "uh", uh);
        // printf("Shooting node %i: [%f, %f, %f, %f, %f, %f] \n", ii, p[0], p[1], p[2], p[3], p[4], p[5]);
    }

    // Set terminal and initial constraints
    // lh: Lower path constraints
    lh[0] = -0.5 * m_admissible_region_x;
    lh[1] = -m_first_admissible_region_y;
    lh[2] = -m_foot_width_x / 2;
    lh[3] = -m_foot_width_y / 2;
    // rh: Upper path constraints
    uh[0] = 0.5 * m_admissible_region_x;
    uh[1] = m_first_admissible_region_y;
    uh[2] = m_foot_width_x / 2;
    uh[3] = m_foot_width_y / 2;
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lh", lh);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "uh", uh);
    //
    // lh[0] = -m_foot_width_x;
    // lh[1] = -m_foot_width_y;
    // lh[2] = -m_foot_width_x*10;
    // lh[3] = -m_foot_width_y*10;

    // uh[0] = m_foot_width_x;
    // uh[1] = m_foot_width_y;
    // uh[2] = m_foot_width_x*10;
    // uh[3] = m_foot_width_y*10;
    // ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "lh", lh);
    // ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "uh", uh);

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

    for (int ii = 0; ii < NX * N; ii++) {
        x_init_input[ii] = xtraj[ii];
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