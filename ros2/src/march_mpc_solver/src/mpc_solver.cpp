/*
 * Project MARCH IX, 2023-2024
 * Author: 
 *  - Alexander James Becoy @alexanderjamesbecoy
 *  - Pleuntje Brons @pleuntjebrons
 */

#include "march_mpc_solver/mpc_solver.hpp"
#include <iostream>

// #define DEBUG

MpcSolver::MpcSolver()
    : m_time_horizon(2.0)
    , m_x_current()
    , m_u_current()
    , m_switch(0)
    , m_current_shooting_node(100)
    , m_timing_value(0)
    , m_current_stance_foot(-1)
    , m_previous_stance_foot(-1)
    , m_step_counter(0) // How many steps are actually set based on the state estimation
    , m_gravity_const(9.81)
    , m_candidate_footsteps()
    , m_reference_stepsize_x(20, 0.0)
    , m_reference_stepsize_y(20, 0.0)
    , m_real_time_com_trajectory_x()
    , m_real_time_com_trajectory_y()
    , m_current_count(-1)
{
    initialize_mpc_params();
    m_x_current.fill(0);
    m_x_trajectory.fill(0);
    m_u_current.fill(0);

    // set_current_com(0.0, 0.33, 0.0, 0.0);
    // set_current_zmp(0.0, 0.33);
    // set_current_foot(0.0, 0.33);
    // set_previous_foot(0.0, 0.0);
    set_current_com(0.0, 0.17, 0.0, 0.0);
    set_current_zmp(0.0, 0.17);
    set_current_foot(0.0, 0.0);
    set_previous_foot(0.0, 0.33);
    set_current_state();
}

int MpcSolver::solve_step()
{
    return solve_zmp_mpc(m_x_current, m_u_current);
}

void MpcSolver::initialize_mpc_params()
{
    // Later, change this to read from a yaml
    m_admissible_region_x = 0.62;
    m_admissible_region_y = 0.10;
    m_foot_width_x = 0.3;
    m_foot_width_y = 0.1;
    m_step_size_x = 0.1;
    m_step_size_y = 0.33;

    m_com_height = 0.6; // Load this from the com position
    m_first_admissible_region_y = 0.01;

    m_switch = 1.0;
    m_current_shooting_node = 100;
    m_timing_value = 0;

    m_number_of_footsteps = 2;
}

void MpcSolver::reset_to_double_stance()
{
    m_current_shooting_node = 100;
    m_step_counter = 0;
    m_current_count = -1;
}

bool MpcSolver::check_zmp_on_foot() const
{
    float zmp_check_margin_y = 1.3;
    float zmp_check_margin_x = 1.3;

    bool x_check = ((m_zmp_current[0] < m_pos_foot_current[0] + m_foot_width_x * zmp_check_margin_x)
        && (m_zmp_current[0] > m_pos_foot_current[0] - m_foot_width_x * zmp_check_margin_x));
    bool y_check = ((m_zmp_current[1] < m_pos_foot_current[1] + m_foot_width_y * zmp_check_margin_y)
        && (m_zmp_current[1] > m_pos_foot_current[1] - m_foot_width_y * zmp_check_margin_y));

    #ifdef DEBUG
    std::cout << "Difference for x is " << m_pos_foot_current[0] - m_zmp_current[0] 
        << " with margin " << m_foot_width_x * zmp_check_margin_x << std::endl;
    std::cout << "Difference for y is " << m_pos_foot_current[1] - m_zmp_current[1] 
        << " with margin " << m_foot_width_y * zmp_check_margin_y << std::endl;
    std::cout << "x_zmp check is " << x_check << std::endl;
    std::cout << "y_zmp check is " << y_check << std::endl;
    #endif
    
    return x_check && y_check;
}

void MpcSolver::set_candidate_footsteps(const geometry_msgs::msg::PoseArray::SharedPtr footsteps)
{
    m_candidate_footsteps.clear();
    for (auto pose : footsteps->poses) {
        m_candidate_footsteps.push_back(pose.position);
    }
}

void MpcSolver::set_reference_stepsize(const std::vector<geometry_msgs::msg::Point>& candidate_footsteps)
{
    m_reference_stepsize_y.clear();
    m_reference_stepsize_x.clear();
    for (long unsigned int i = 0; i < candidate_footsteps.size() - 1; i++) {
        m_reference_stepsize_x.push_back(candidate_footsteps[i + 1].x - candidate_footsteps[i].x);
        m_reference_stepsize_y.push_back(candidate_footsteps[i + 1].y - candidate_footsteps[i].y);
    }
}

void MpcSolver::update_current_foot()
{
    if (m_step_counter == 0) {
        // m_pos_foot_current[0] = m_x_trajectory[6 + NX];
        m_pos_foot_current[0] = 0.0; // at the start, the CoM is about 0.11 meters in the positive direction, because
                                     // the 0 is from the right ankle
        m_pos_foot_current[1] = m_x_trajectory[8 + NX];
    } else if (m_step_counter != 0 && m_current_shooting_node == 1) {
        m_pos_foot_current[0] = m_x_trajectory[6 + NX] - m_x_trajectory[6];
        m_pos_foot_current[1] = m_x_trajectory[8 + NX];
    }
}

inline int MpcSolver::solve_zmp_mpc(
    std::array<double, NX>& x_init_input, std::array<double, NU * ZMP_PENDULUM_ODE_N>& u_current)
{
    // geometry_msgs::msg::Pose left_foot_pose = m_foot_positions.poses[0];
    // geometry_msgs::msg::Pose right_foot_pose = m_foot_positions.poses[1];

    // double left_foot_x = left_foot_pose.position.x;
    // double left_foot_y = left_foot_pose.position.y;
    // double right_foot_x = right_foot_pose.position.x;
    // double right_foot_y = right_foot_pose.position.y;

    // int current_stance_leg = m_current_stance_leg;
    // int next_stance_leg = m_next_stance_leg;

    ZMP_pendulum_ode_solver_capsule* acados_ocp_capsule = ZMP_pendulum_ode_acados_create_capsule();
    // there is an opportunity to change the number of shooting intervals in C without new code generation
    int N = ZMP_PENDULUM_ODE_N;

    // allocate the array and fill it accordingly
    double* new_time_steps = NULL;
    //printf("Shooting nodes: %i\n", N);
    // First, we do some checks so the mpc works
    if ((N - 1) % m_number_of_footsteps != 0) {
        return ACADOS_READY;
    }

    int status = ZMP_pendulum_ode_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);

    if (status) {
        #ifdef DEBUG
        std::cout << "ZMP_pendulum_ode_acados_create() returned status " << status << ". Exiting." << std::endl;
        #endif
        exit(1);
    }

    ocp_nlp_config* nlp_config = ZMP_pendulum_ode_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims* nlp_dims = ZMP_pendulum_ode_acados_get_nlp_dims(acados_ocp_capsule);
    ocp_nlp_in* nlp_in = ZMP_pendulum_ode_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out* nlp_out = ZMP_pendulum_ode_acados_get_nlp_out(acados_ocp_capsule);
    ocp_nlp_solver* nlp_solver = ZMP_pendulum_ode_acados_get_nlp_solver(acados_ocp_capsule);
    void* nlp_opts = ZMP_pendulum_ode_acados_get_nlp_opts(acados_ocp_capsule);

    // initial condition
    int idxbx0[NBX0];   // index of x for bounds
    double lbx0[NBX0];  // lower bound for x
    double ubx0[NBX0];  // upper bound for x

    for (int ii = 0; ii < NBX0; ii++) {
        idxbx0[ii] = ii;
        lbx0[ii] = x_init_input[ii];
        ubx0[ii] = x_init_input[ii];
    }

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

    // We define our constraints here
    double lh[4];   // lh: Lower path constraints
    double uh[4];   // rh: Upper path constraints

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
    for (int i = 0; i < NX; i++) {
        x_init[i] = x_init_input[i];
    }

    // initial value for control input
    double u0[NU];
    for (int i = 0; i < NU; i++) {
        u0[i] = 0.0;
    }

    // set parameters
    // WE NEED TO SET OUR PARAMETERS HERE
    double p[NP];
    p[0] = 0;   // p[0] : Planned footstep width;
    p[1] = 0;   // p[1] : Planned footstep height;
    p[2] = 0;   // p[2] : Switch variable(0 at all times, 1 when we want to take a step);
    p[3] = 0;   // p[3] : moving foot constraint timing variable(0-1);
    p[4] = 0;   // p[4] : Periodic tail constraint;
    p[5] = 0;   // p[5] : RK4 constraint (always 0);
    p[6] = 0;   // p[6] : RK4 constraint (always 0);

    double dt = 0.0 + (m_time_horizon) / (N - 1);
    // If the footstep is the left foot or the right foot(left is -1, right is 1)

    m_timing_value = 0.0;
    m_switch = 1.0 / dt;
    // The step number
    int step_number = 0; // the virtual step number for the MPC to predict a horizon
    float step_duration = 0.6; // Set this to swing leg_duration, in percentage, so 60% of a step is single stance.
    float step_duration_factor = 1.0 / step_duration;

    #ifdef DEBUG
    // check footstep planner references
     std::cout << "Vector elements: ";
     for (const auto& element : m_reference_stepsize_x) {
         printf("element x is %f\n", element);
     }
     for (const auto& element : m_reference_stepsize_y) {
         printf("element y is %f\n", element);
     }
     std::cout << std::endl;
    #endif

    // When a new step is set, take the next reference step from the footstep planner generated trajectory

    // To decide what the timing value is depending on the current shooting node is

    m_current_shooting_node = m_current_shooting_node % (((N - 1)) / (m_number_of_footsteps));

    // // check if swing leg is done for left foot as stance leg and right leg as swing leg
    // if (m_current_shooting_node == step_duration * (((N - 1)) / (m_number_of_footsteps)) + 1 && m_current_count == -1
    //     && m_right_foot_on_ground == true) {
    //     printf("passed the right foot on ground check \n");
    //     m_right_foot_on_ground = false;
    // } else if (m_current_shooting_node == step_duration * (((N - 1)) / (m_number_of_footsteps)) + 1
    //     && m_current_count == 1 && m_left_foot_on_ground == true) {
    //     printf("passed the left foot on ground check \n");
    //     m_left_foot_on_ground = false;
    // } else if (m_current_shooting_node == step_duration * (((N - 1)) / (m_number_of_footsteps)) + 1) {
    //     printf("did not pass the foot on ground check \n");
    // m_current_shooting_node--;
    // }

    // only change the initial count when a new footstep has to be set and check if the weight shift is done by checking
    // the current stance foot and ZMP location based on a margin. (The ZMP has to be on the new stance foot)
    // RCLCPP_INFO(rclcpp::get_logger(""), "x zmp is %f", m_zmp_current[0]);
    // RCLCPP_INFO(rclcpp::get_logger(""), "y zmp is %f", m_zmp_current[1]);
    // RCLCPP_INFO(rclcpp::get_logger(""), "Stance foot is %d", m_current_stance_foot);
    if (m_current_shooting_node == 0 && m_current_stance_foot == -1 && m_current_count == 1 && check_zmp_on_foot()

        || m_current_shooting_node == 0 && m_current_stance_foot == 1 && m_current_count == -1 && check_zmp_on_foot()) {
        // m_current_count = m_current_stance_foot;
        // m_step_counter++;
        // RCLCPP_INFO(rclcpp::get_logger(""), "weight shift is complete \n");
        // printf("now going into current_shooting node %i\n", m_current_shooting_node);
    } else if (m_current_shooting_node == 0) {
        // m_current_shooting_node--; // stance foot is not working so without check we have to switch the stance leg
        // manually and increment the step counter still
        m_current_count = -m_current_count; // turn off if pressure sole on
        m_step_counter++; // turn off if pressure sole on
    }
    // RCLCPP_INFO(rclcpp::get_logger("Internal stance foot"), "current count is %i \n", m_current_count);

    //printf("step_counter %i\n", m_step_counter);
    //printf("current stance foot is %i\n", m_current_stance_foot);
    //printf("current count is %i\n", m_current_count);

    // correction so that current shooting node doesn't turn negative
    if (m_current_shooting_node == -1) {
        m_current_shooting_node = (((N - 1)) / (m_number_of_footsteps)) - 1;
    }

    int count = m_current_count;

    if (m_current_shooting_node != 0 && step_duration * ((N - 1) / m_number_of_footsteps) < m_current_shooting_node
        && m_current_shooting_node < ((N - 1)) / m_number_of_footsteps) {
        m_timing_value = (m_current_shooting_node - step_duration * ((N - 1) / m_number_of_footsteps))
            * (1 / (1 - step_duration)) / (((N - 1)) / (m_number_of_footsteps));
        count = -count;
        step_number += 1;
        //printf("now going into current_shooting node %i\n", m_current_shooting_node);
        //printf("with timing value %f\n", m_timing_value);
    } else if (m_current_shooting_node != 0
        && m_current_shooting_node < step_duration * ((N - 1) / m_number_of_footsteps)) {
        m_timing_value = 0;
        count = -count;
        step_number += 1;
        //printf("now going into current_shooting node %i\n", m_current_shooting_node);
        //printf("with timing value in else if %f\n", m_timing_value);

    } else {
        ;
    }

    // ii is defined as the current stage
    for (int ii = 0; ii < N; ii++) {
        if (((ii + m_current_shooting_node) % ((N - 1) / m_number_of_footsteps) == 0)) {

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
            count = -count;
            m_timing_value = 1;

            p[0] = m_reference_stepsize_x[step_number + m_step_counter] / dt;
            p[1] = m_reference_stepsize_y[step_number + m_step_counter] / dt;
            p[2] = m_switch;
            p[3] = m_timing_value;
            p[4] = 0;
            step_number += 1;
            //printf("Shooting node %i: [%f, %f, %f, %f, %f] \n", ii, p[0], p[1], p[2], p[3], p[4]);

            ZMP_pendulum_ode_acados_update_params(acados_ocp_capsule, ii, p, NP);
            m_timing_value = 0;
            // printf("Timing value t is %f\n", m_timing_value);

        } else if ((step_number - 1) * ((N - 1) / m_number_of_footsteps) < ii + m_current_shooting_node
            && ii + m_current_shooting_node <= (step_number - 1) * ((N - 1) / m_number_of_footsteps)
                    + ((N - 1) / m_number_of_footsteps / step_duration_factor)) {
            // KEEP SHADOW FOOT ON THE STANCE LEG, keeps timing factor at 0 for the stance leg duration
            m_timing_value = 0;
            p[0] = 0;
            p[1] = 0;
            p[2] = 0;
            p[3] = m_timing_value;
            p[4] = 0;
            ZMP_pendulum_ode_acados_update_params(acados_ocp_capsule, ii, p, NP);

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
            // printf("elif shooting node ii + current = %i\n", ii+m_current_shooting_node);
            // printf("Between %d\n", (step_number-1)*((N-1)/m_number_of_footsteps));
            // printf("and %f\n",(step_number - 1) * ((N - 1) / m_number_of_footsteps) + ((N - 1) /
            // m_number_of_footsteps / step_duration_factor));

            // for periodic tail constraint
        } else if (ii + m_current_shooting_node
            == (2 * (m_current_shooting_node + (N - 1) / m_number_of_footsteps) - 1)) {
            m_timing_value += (1 / (1 - step_duration)) / (((N - 1)) / (m_number_of_footsteps));

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
            p[0] = 0;
            p[1] = 0;
            p[2] = 0;
            p[3] = m_timing_value;
            p[4] = 1;

            ZMP_pendulum_ode_acados_update_params(acados_ocp_capsule, ii, p, NP);

        } else {
            // Standard weight shift
            // m_timing_value += 1.0 / (((N - 1) - m_number_of_footsteps) / (m_number_of_footsteps));
            m_timing_value += (1 / (1 - step_duration)) / (((N - 1)) / (m_number_of_footsteps));

            p[0] = 0;
            p[1] = 0;
            p[2] = 0;
            p[3] = m_timing_value;
            p[4] = 0;
            ZMP_pendulum_ode_acados_update_params(acados_ocp_capsule, ii, p, NP);
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
            // printf("Else Timing value t is %f\n", m_timing_value);
        }
        // printf("current_shooting node is %i\n", ii + m_current_shooting_node);
        // printf("tming value node is %f\n", m_timing_value);

        // printf("Shooting node %i: [%f, %f, %f, %f, %f] \n", ii, p[0], p[1], p[2], p[3], p[4]);
    }
    // printf("current_shooting node is %i\n", m_current_shooting_node);
    // RCLCPP_INFO(rclcpp::get_logger(""), "Current shooting node is %i", m_current_shooting_node);
    // Set terminal and initial constraints

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

    // #ifdef DEBUG
    // /* print solution and statistics */
    // for (int ii = 0; ii <= nlp_dims->N; ii++)
    //     ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &xtraj[ii * NX]);
    // for (int ii = 0; ii < nlp_dims->N; ii++)
    //     ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &utraj[ii * NU]);

    // std::cout << "--- xtraj ---" << std::endl;
    // d_print_exp_tran_mat(NX, N + 1, xtraj, NX);

    // std::cout << "--- utraj ---" << std::endl;
    // d_print_exp_tran_mat(NU, N, utraj, NU);

    // ocp_nlp_out_print(nlp_solver->dims, nlp_out);

    // std::cout << "The current state is " << std::endl;
    // for (long unsigned int i = 0; i < NX; i++) {
    //     //printf(" %f", xtraj[i]);
    //     std::cout << xtraj[i] << " ";
    // }

    // std::cout << "Solved ocp " << NTIMINGS << " times, solution printed above" << std::endl;
    // #endif

    if (status == ACADOS_SUCCESS) {
        #ifdef DEBUG
        std::cout << "ZMP_pendulum_ode_acados_solve() SUCCESS!" << std::endl;
        #endif

        for (int ii = 0; ii < nlp_dims->N; ii++) {
            u_current[ii] = utraj[ii];
        }

        for (int ii = 0; ii < NX; ii++) {
            x_init_input[ii] = xtraj[NX + ii];
        }

        std::copy(xtraj, xtraj + NX * ZMP_PENDULUM_ODE_N, m_x_trajectory.begin());
    } else {
        #ifdef DEBUG
        std::cout << "ZMP_pendulum_ode_acados_solve() failed with status " << status << "." << std::endl;
        #endif
    }

    // here, we copy our array into the std::array

    // for (int ii = 0; ii < nlp_dims->N; ii++) {
    //         u_current[ii] = utraj[ii];
    //     }

    // for (int ii = 0; ii < NX; ii++) {
    //        x_init_input[ii] = xtraj[NX + ii];
    //     }
    // Take solution from trajectory for visualization
    // m_real_time_com_trajectory_x.empty();
    // m_real_time_com_trajectory_y.empty();
    // std::copy(xtraj, xtraj + NX * ZMP_PENDULUM_ODE_N, m_x_trajectory.begin());
    // for (int ii = 0; ii < NX * (N + 1); ii += NX) {
    // m_x_trajectory[ii] = xtraj[ii];
    // m_real_time_com_trajectory_x.push_back(xtraj[ii]);
    // m_real_time_com_trajectory_y.push_back(xtraj[ii+3]);
    // }

    // printf("m_x_trajectory %f \n", m_x_trajectory[3+12]);
    // printf("m_x_trajectory is %ld \n", m_x_trajectory.size());

    // m_real_time_com_trajectory_x.push_back(xtraj[0]);
    // m_real_time_com_trajectory_y.push_back(xtraj[3]);

    // for (auto element : m_x_trajectory){
    //     printf("real time trajectory is %f\n", element);
    // }

    // get solution
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter);

    // ZMP_pendulum_ode_acados_print_stats(acados_ocp_capsule);

    // printf("the solution x is %f\n:", m_x_trajectory[6+12]);
    // printf("the solution y is %f\n:", m_x_trajectory[8+12]);
    //printf("\nSolver info:\n");
    //printf(" SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n", sqp_iter, NTIMINGS, min_time * 1000,
        //kkt_norm_inf);

    // free solver
    status = ZMP_pendulum_ode_acados_free(acados_ocp_capsule);
    if (status) {
        //printf("ZMP_pendulum_ode_acados_free() returned status %d. \n", status);
    }
    // free solver capsule
    status = ZMP_pendulum_ode_acados_free_capsule(acados_ocp_capsule);
    if (status) {
        //printf("ZMP_pendulum_ode_acados_free_capsule() returned status %d. \n", status);
    }

    return status;
}