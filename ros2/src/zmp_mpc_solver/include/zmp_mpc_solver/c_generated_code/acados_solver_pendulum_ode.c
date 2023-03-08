/*
 * Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
 * Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
 * Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
 * Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf, Moritz Diehl
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

// standard
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
// acados
// #include "acados/utils/print.h"
#include "acados_c/external_function_interface.h"
#include "acados_c/ocp_nlp_interface.h"

// example specific
#include "pendulum_ode_model/pendulum_ode_model.h"

#include "pendulum_ode_cost/pendulum_ode_external_cost.h"
#include "pendulum_ode_cost/pendulum_ode_external_cost_0.h"
#include "pendulum_ode_cost/pendulum_ode_external_cost_e.h"

#include "acados_solver_pendulum_ode.h"

#define NX PENDULUM_ODE_NX
#define NZ PENDULUM_ODE_NZ
#define NU PENDULUM_ODE_NU
#define NP PENDULUM_ODE_NP
#define NBX PENDULUM_ODE_NBX
#define NBX0 PENDULUM_ODE_NBX0
#define NBU PENDULUM_ODE_NBU
#define NSBX PENDULUM_ODE_NSBX
#define NSBU PENDULUM_ODE_NSBU
#define NSH PENDULUM_ODE_NSH
#define NSG PENDULUM_ODE_NSG
#define NSPHI PENDULUM_ODE_NSPHI
#define NSHN PENDULUM_ODE_NSHN
#define NSGN PENDULUM_ODE_NSGN
#define NSPHIN PENDULUM_ODE_NSPHIN
#define NSBXN PENDULUM_ODE_NSBXN
#define NS PENDULUM_ODE_NS
#define NSN PENDULUM_ODE_NSN
#define NG PENDULUM_ODE_NG
#define NBXN PENDULUM_ODE_NBXN
#define NGN PENDULUM_ODE_NGN
#define NY0 PENDULUM_ODE_NY0
#define NY PENDULUM_ODE_NY
#define NYN PENDULUM_ODE_NYN
// #define N      PENDULUM_ODE_N
#define NH PENDULUM_ODE_NH
#define NPHI PENDULUM_ODE_NPHI
#define NHN PENDULUM_ODE_NHN
#define NPHIN PENDULUM_ODE_NPHIN
#define NR PENDULUM_ODE_NR

// ** solver data **

pendulum_ode_solver_capsule* pendulum_ode_acados_create_capsule(void)
{
    void* capsule_mem = malloc(sizeof(pendulum_ode_solver_capsule));
    pendulum_ode_solver_capsule* capsule = (pendulum_ode_solver_capsule*)capsule_mem;

    return capsule;
}

int pendulum_ode_acados_free_capsule(pendulum_ode_solver_capsule* capsule)
{
    free(capsule);
    return 0;
}

int pendulum_ode_acados_create(pendulum_ode_solver_capsule* capsule)
{
    int N_shooting_intervals = PENDULUM_ODE_N;
    double* new_time_steps = NULL; // NULL -> don't alter the code generated time-steps
    return pendulum_ode_acados_create_with_discretization(capsule, N_shooting_intervals, new_time_steps);
}

int pendulum_ode_acados_update_time_steps(pendulum_ode_solver_capsule* capsule, int N, double* new_time_steps)
{
    if (N != capsule->nlp_solver_plan->N) {
        fprintf(stderr,
            "pendulum_ode_acados_update_time_steps: given number of time steps (= %d) "
            "differs from the currently allocated number of "
            "time steps (= %d)!\n"
            "Please recreate with new discretization and provide a new vector of time_stamps!\n",
            N, capsule->nlp_solver_plan->N);
        return 1;
    }

    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_in* nlp_in = capsule->nlp_in;

    for (int i = 0; i < N; i++) {
        ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &new_time_steps[i]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &new_time_steps[i]);
    }
    return 0;
}

/**
 * Internal function for pendulum_ode_acados_create: step 1
 */
void pendulum_ode_acados_create_1_set_plan(ocp_nlp_plan_t* nlp_solver_plan, const int N)
{
    assert(N == nlp_solver_plan->N);

    /************************************************
     *  plan
     ************************************************/
    nlp_solver_plan->nlp_solver = SQP;

    nlp_solver_plan->ocp_qp_solver_plan.qp_solver = FULL_CONDENSING_QPOASES;

    nlp_solver_plan->nlp_cost[0] = EXTERNAL;
    for (int i = 1; i < N; i++)
        nlp_solver_plan->nlp_cost[i] = EXTERNAL;

    nlp_solver_plan->nlp_cost[N] = EXTERNAL;

    for (int i = 0; i < N; i++) {
        nlp_solver_plan->nlp_dynamics[i] = CONTINUOUS_MODEL;
        nlp_solver_plan->sim_solver_plan[i].sim_solver = IRK;
    }

    for (int i = 0; i < N; i++) {
        nlp_solver_plan->nlp_constraints[i] = BGH;
    }
    nlp_solver_plan->nlp_constraints[N] = BGH;
}

/**
 * Internal function for pendulum_ode_acados_create: step 2
 */
ocp_nlp_dims* pendulum_ode_acados_create_2_create_and_set_dimensions(pendulum_ode_solver_capsule* capsule)
{
    ocp_nlp_plan_t* nlp_solver_plan = capsule->nlp_solver_plan;
    const int N = nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;

/************************************************
 *  dimensions
 ************************************************/
#define NINTNP1MEMS 17
    int* intNp1mem = (int*)malloc((N + 1) * sizeof(int) * NINTNP1MEMS);

    int* nx = intNp1mem + (N + 1) * 0;
    int* nu = intNp1mem + (N + 1) * 1;
    int* nbx = intNp1mem + (N + 1) * 2;
    int* nbu = intNp1mem + (N + 1) * 3;
    int* nsbx = intNp1mem + (N + 1) * 4;
    int* nsbu = intNp1mem + (N + 1) * 5;
    int* nsg = intNp1mem + (N + 1) * 6;
    int* nsh = intNp1mem + (N + 1) * 7;
    int* nsphi = intNp1mem + (N + 1) * 8;
    int* ns = intNp1mem + (N + 1) * 9;
    int* ng = intNp1mem + (N + 1) * 10;
    int* nh = intNp1mem + (N + 1) * 11;
    int* nphi = intNp1mem + (N + 1) * 12;
    int* nz = intNp1mem + (N + 1) * 13;
    int* ny = intNp1mem + (N + 1) * 14;
    int* nr = intNp1mem + (N + 1) * 15;
    int* nbxe = intNp1mem + (N + 1) * 16;

    for (int i = 0; i < N + 1; i++) {
        // common
        nx[i] = NX;
        nu[i] = NU;
        nz[i] = NZ;
        ns[i] = NS;
        // cost
        ny[i] = NY;
        // constraints
        nbx[i] = NBX;
        nbu[i] = NBU;
        nsbx[i] = NSBX;
        nsbu[i] = NSBU;
        nsg[i] = NSG;
        nsh[i] = NSH;
        nsphi[i] = NSPHI;
        ng[i] = NG;
        nh[i] = NH;
        nphi[i] = NPHI;
        nr[i] = NR;
        nbxe[i] = 0;
    }

    // for initial state
    nbx[0] = NBX0;
    nsbx[0] = 0;
    ns[0] = NS - NSBX;
    nbxe[0] = 2;
    ny[0] = NY0;

    // terminal - common
    nu[N] = 0;
    nz[N] = 0;
    ns[N] = NSN;
    // cost
    ny[N] = NYN;
    // constraint
    nbx[N] = NBXN;
    nbu[N] = 0;
    ng[N] = NGN;
    nh[N] = NHN;
    nphi[N] = NPHIN;
    nr[N] = 0;

    nsbx[N] = NSBXN;
    nsbu[N] = 0;
    nsg[N] = NSGN;
    nsh[N] = NSHN;
    nsphi[N] = NSPHIN;

    /* create and set ocp_nlp_dims */
    ocp_nlp_dims* nlp_dims = ocp_nlp_dims_create(nlp_config);

    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nx", nx);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nu", nu);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nz", nz);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "ns", ns);

    for (int i = 0; i <= N; i++) {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbx", &nbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbu", &nbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbx", &nsbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbu", &nsbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "ng", &ng[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsg", &nsg[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbxe", &nbxe[i]);
    }

    for (int i = 0; i < N; i++) { }
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nh", &nh[N]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nsh", &nsh[N]);
    free(intNp1mem);
    return nlp_dims;
}

/**
 * Internal function for pendulum_ode_acados_create: step 3
 */
void pendulum_ode_acados_create_3_create_and_set_functions(pendulum_ode_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;

    /************************************************
     *  external functions
     ************************************************/

#define MAP_CASADI_FNC(__CAPSULE_FNC__, __MODEL_BASE_FNC__)                                                            \
    do {                                                                                                               \
        capsule->__CAPSULE_FNC__.casadi_fun = &__MODEL_BASE_FNC__;                                                     \
        capsule->__CAPSULE_FNC__.casadi_n_in = &__MODEL_BASE_FNC__##_n_in;                                             \
        capsule->__CAPSULE_FNC__.casadi_n_out = &__MODEL_BASE_FNC__##_n_out;                                           \
        capsule->__CAPSULE_FNC__.casadi_sparsity_in = &__MODEL_BASE_FNC__##_sparsity_in;                               \
        capsule->__CAPSULE_FNC__.casadi_sparsity_out = &__MODEL_BASE_FNC__##_sparsity_out;                             \
        capsule->__CAPSULE_FNC__.casadi_work = &__MODEL_BASE_FNC__##_work;                                             \
        external_function_param_casadi_create(&capsule->__CAPSULE_FNC__, 0);                                           \
    } while (false)

    // implicit dae
    capsule->impl_dae_fun = (external_function_param_casadi*)malloc(sizeof(external_function_param_casadi) * N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(impl_dae_fun[i], pendulum_ode_impl_dae_fun);
    }

    capsule->impl_dae_fun_jac_x_xdot_z
        = (external_function_param_casadi*)malloc(sizeof(external_function_param_casadi) * N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(impl_dae_fun_jac_x_xdot_z[i], pendulum_ode_impl_dae_fun_jac_x_xdot_z);
    }

    capsule->impl_dae_jac_x_xdot_u_z
        = (external_function_param_casadi*)malloc(sizeof(external_function_param_casadi) * N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(impl_dae_jac_x_xdot_u_z[i], pendulum_ode_impl_dae_jac_x_xdot_u_z);
    }

    // external cost
    MAP_CASADI_FNC(ext_cost_0_fun, pendulum_ode_cost_ext_cost_0_fun);

    // external cost
    MAP_CASADI_FNC(ext_cost_0_fun_jac, pendulum_ode_cost_ext_cost_0_fun_jac);

    // external cost
    MAP_CASADI_FNC(ext_cost_0_fun_jac_hess, pendulum_ode_cost_ext_cost_0_fun_jac_hess);
    // external cost
    capsule->ext_cost_fun = (external_function_param_casadi*)malloc(sizeof(external_function_param_casadi) * N);
    for (int i = 0; i < N - 1; i++) {
        MAP_CASADI_FNC(ext_cost_fun[i], pendulum_ode_cost_ext_cost_fun);
    }

    capsule->ext_cost_fun_jac = (external_function_param_casadi*)malloc(sizeof(external_function_param_casadi) * N);
    for (int i = 0; i < N - 1; i++) {
        MAP_CASADI_FNC(ext_cost_fun_jac[i], pendulum_ode_cost_ext_cost_fun_jac);
    }

    capsule->ext_cost_fun_jac_hess
        = (external_function_param_casadi*)malloc(sizeof(external_function_param_casadi) * N);
    for (int i = 0; i < N - 1; i++) {
        MAP_CASADI_FNC(ext_cost_fun_jac_hess[i], pendulum_ode_cost_ext_cost_fun_jac_hess);
    }
    // external cost - function
    MAP_CASADI_FNC(ext_cost_e_fun, pendulum_ode_cost_ext_cost_e_fun);

    // external cost - jacobian
    MAP_CASADI_FNC(ext_cost_e_fun_jac, pendulum_ode_cost_ext_cost_e_fun_jac);

    // external cost - hessian
    MAP_CASADI_FNC(ext_cost_e_fun_jac_hess, pendulum_ode_cost_ext_cost_e_fun_jac_hess);

#undef MAP_CASADI_FNC
}

/**
 * Internal function for pendulum_ode_acados_create: step 4
 */
void pendulum_ode_acados_create_4_set_default_parameters(pendulum_ode_solver_capsule* capsule)
{
    // no parameters defined
}

/**
 * Internal function for pendulum_ode_acados_create: step 5
 */
void pendulum_ode_acados_create_5_set_nlp_in(pendulum_ode_solver_capsule* capsule, const int N, double* new_time_steps)
{
    assert(N == capsule->nlp_solver_plan->N);
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;

    /************************************************
     *  nlp_in
     ************************************************/
    //    ocp_nlp_in * nlp_in = ocp_nlp_in_create(nlp_config, nlp_dims);
    //    capsule->nlp_in = nlp_in;
    ocp_nlp_in* nlp_in = capsule->nlp_in;

    // set up time_steps

    if (new_time_steps) {
        pendulum_ode_acados_update_time_steps(capsule, N, new_time_steps);
    } else { // all time_steps are identical
        double time_step = 0.04285714285714286;
        for (int i = 0; i < N; i++) {
            ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &time_step);
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &time_step);
        }
    }

    /**** Dynamics ****/
    for (int i = 0; i < N; i++) {
        ocp_nlp_dynamics_model_set(nlp_config, nlp_dims, nlp_in, i, "impl_dae_fun", &capsule->impl_dae_fun[i]);
        ocp_nlp_dynamics_model_set(
            nlp_config, nlp_dims, nlp_in, i, "impl_dae_fun_jac_x_xdot_z", &capsule->impl_dae_fun_jac_x_xdot_z[i]);
        ocp_nlp_dynamics_model_set(
            nlp_config, nlp_dims, nlp_in, i, "impl_dae_jac_x_xdot_u", &capsule->impl_dae_jac_x_xdot_u_z[i]);
    }

    /**** Cost ****/
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "ext_cost_fun", &capsule->ext_cost_0_fun);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "ext_cost_fun_jac", &capsule->ext_cost_0_fun_jac);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "ext_cost_fun_jac_hess", &capsule->ext_cost_0_fun_jac_hess);
    for (int i = 1; i < N; i++) {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "ext_cost_fun", &capsule->ext_cost_fun[i - 1]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "ext_cost_fun_jac", &capsule->ext_cost_fun_jac[i - 1]);
        ocp_nlp_cost_model_set(
            nlp_config, nlp_dims, nlp_in, i, "ext_cost_fun_jac_hess", &capsule->ext_cost_fun_jac_hess[i - 1]);
    }

    // terminal cost
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "ext_cost_fun", &capsule->ext_cost_e_fun);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "ext_cost_fun_jac", &capsule->ext_cost_e_fun_jac);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "ext_cost_fun_jac_hess", &capsule->ext_cost_e_fun_jac_hess);

    /**** Constraints ****/

    // bounds for initial stage
    // x0
    int* idxbx0 = malloc(NBX0 * sizeof(int));
    idxbx0[0] = 0;
    idxbx0[1] = 1;

    double* lubx0 = calloc(2 * NBX0, sizeof(double));
    double* lbx0 = lubx0;
    double* ubx0 = lubx0 + NBX0;
    // change only the non-zero elements:
    lbx0[0] = 0.3;
    ubx0[0] = 0.3;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);
    free(idxbx0);
    free(lubx0);
    // idxbxe_0
    int* idxbxe_0 = malloc(2 * sizeof(int));

    idxbxe_0[0] = 0;
    idxbxe_0[1] = 1;
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbxe", idxbxe_0);
    free(idxbxe_0);

    /* constraints that are the same for initial and intermediate */
    // u
    int* idxbu = malloc(NBU * sizeof(int));

    idxbu[0] = 0;
    double* lubu = calloc(2 * NBU, sizeof(double));
    double* lbu = lubu;
    double* ubu = lubu + NBU;

    lbu[0] = -1.57;
    ubu[0] = 1.57;

    for (int i = 0; i < N; i++) {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbu", idxbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubu", ubu);
    }
    free(idxbu);
    free(lubu);

    /* terminal constraints */
}

/**
 * Internal function for pendulum_ode_acados_create: step 6
 */
void pendulum_ode_acados_create_6_set_opts(pendulum_ode_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    void* nlp_opts = capsule->nlp_opts;

    /************************************************
     *  opts
     ************************************************/

    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "globalization", "fixed_step");
    int full_step_dual = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "full_step_dual", &full_step_dual);

    // set collocation type (relevant for implicit integrators)
    sim_collocation_type collocation_type = GAUSS_LEGENDRE;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_collocation_type", &collocation_type);

    // set up sim_method_num_steps
    // all sim_method_num_steps are identical
    int sim_method_num_steps = 1;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_num_steps", &sim_method_num_steps);

    // set up sim_method_num_stages
    // all sim_method_num_stages are identical
    int sim_method_num_stages = 4;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_num_stages", &sim_method_num_stages);

    int newton_iter_val = 3;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_newton_iter", &newton_iter_val);

    // set up sim_method_jac_reuse
    bool tmp_bool = (bool)0;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_jac_reuse", &tmp_bool);

    double nlp_solver_step_length = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "step_length", &nlp_solver_step_length);

    double levenberg_marquardt = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "levenberg_marquardt", &levenberg_marquardt);

    /* options QP solver */

    int nlp_solver_ext_qp_res = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "ext_qp_res", &nlp_solver_ext_qp_res);

    // set SQP specific options
    double nlp_solver_tol_stat = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_stat", &nlp_solver_tol_stat);

    double nlp_solver_tol_eq = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_eq", &nlp_solver_tol_eq);

    double nlp_solver_tol_ineq = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_ineq", &nlp_solver_tol_ineq);

    double nlp_solver_tol_comp = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_comp", &nlp_solver_tol_comp);

    int nlp_solver_max_iter = 1000;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "max_iter", &nlp_solver_max_iter);

    int initialize_t_slacks = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "initialize_t_slacks", &initialize_t_slacks);

    int qp_solver_iter_max = 50;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_iter_max", &qp_solver_iter_max);

    int print_level = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "print_level", &print_level);

    int ext_cost_num_hess = 0;
    for (int i = 0; i < N; i++) {
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "cost_numerical_hessian", &ext_cost_num_hess);
    }
    ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, N, "cost_numerical_hessian", &ext_cost_num_hess);
}

/**
 * Internal function for pendulum_ode_acados_create: step 7
 */
void pendulum_ode_acados_create_7_set_nlp_out(pendulum_ode_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;

    // initialize primal solution
    double* xu0 = calloc(NX + NU, sizeof(double));
    double* x0 = xu0;

    // initialize with x0

    x0[0] = 0.3;

    double* u0 = xu0 + NX;

    for (int i = 0; i < N; i++) {
        // x0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x0);
        // u0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
    }
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x0);
    free(xu0);
}

/**
 * Internal function for pendulum_ode_acados_create: step 8
 */
// void pendulum_ode_acados_create_8_create_solver(pendulum_ode_solver_capsule* capsule)
//{
//    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts);
//}

/**
 * Internal function for pendulum_ode_acados_create: step 9
 */
int pendulum_ode_acados_create_9_precompute(pendulum_ode_solver_capsule* capsule)
{
    int status = ocp_nlp_precompute(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    if (status != ACADOS_SUCCESS) {
        printf("\nocp_nlp_precompute failed!\n\n");
        exit(1);
    }

    return status;
}

int pendulum_ode_acados_create_with_discretization(pendulum_ode_solver_capsule* capsule, int N, double* new_time_steps)
{
    // If N does not match the number of shooting intervals used for code generation, new_time_steps must be given.
    if (N != PENDULUM_ODE_N && !new_time_steps) {
        fprintf(stderr,
            "pendulum_ode_acados_create_with_discretization: new_time_steps is NULL "
            "but the number of shooting intervals (= %d) differs from the number of "
            "shooting intervals (= %d) during code generation! Please provide a new vector of time_stamps!\n",
            N, PENDULUM_ODE_N);
        return 1;
    }

    // number of expected runtime parameters
    capsule->nlp_np = NP;

    // 1) create and set nlp_solver_plan; create nlp_config
    capsule->nlp_solver_plan = ocp_nlp_plan_create(N);
    pendulum_ode_acados_create_1_set_plan(capsule->nlp_solver_plan, N);
    capsule->nlp_config = ocp_nlp_config_create(*capsule->nlp_solver_plan);

    // 3) create and set dimensions
    capsule->nlp_dims = pendulum_ode_acados_create_2_create_and_set_dimensions(capsule);
    pendulum_ode_acados_create_3_create_and_set_functions(capsule);

    // 4) set default parameters in functions
    pendulum_ode_acados_create_4_set_default_parameters(capsule);

    // 5) create and set nlp_in
    capsule->nlp_in = ocp_nlp_in_create(capsule->nlp_config, capsule->nlp_dims);
    pendulum_ode_acados_create_5_set_nlp_in(capsule, N, new_time_steps);

    // 6) create and set nlp_opts
    capsule->nlp_opts = ocp_nlp_solver_opts_create(capsule->nlp_config, capsule->nlp_dims);
    pendulum_ode_acados_create_6_set_opts(capsule);

    // 7) create and set nlp_out
    // 7.1) nlp_out
    capsule->nlp_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    // 7.2) sens_out
    capsule->sens_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    pendulum_ode_acados_create_7_set_nlp_out(capsule);

    // 8) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts);
    // pendulum_ode_acados_create_8_create_solver(capsule);

    // 9) do precomputations
    int status = pendulum_ode_acados_create_9_precompute(capsule);
    return status;
}

/**
 * This function is for updating an already initialized solver with a different number of qp_cond_N. It is useful for
 * code reuse after code export.
 */
int pendulum_ode_acados_update_qp_solver_cond_N(pendulum_ode_solver_capsule* capsule, int qp_solver_cond_N)
{
    printf("\nacados_update_qp_solver_cond_N() failed, since no partial condensing solver is used!\n\n");
    // Todo: what is an adequate behavior here?
    exit(1);
    return -1;
}

int pendulum_ode_acados_reset(pendulum_ode_solver_capsule* capsule, int reset_qp_solver_mem)
{

    // set initialization to all zeros

    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;
    ocp_nlp_in* nlp_in = capsule->nlp_in;
    ocp_nlp_solver* nlp_solver = capsule->nlp_solver;

    int nx, nu, nv, ns, nz, ni, dim;

    double* buffer = calloc(
        NX + NU + NZ + 2 * NS + 2 * NSN + NBX + NBU + NG + NH + NPHI + NBX0 + NBXN + NHN + NPHIN + NGN, sizeof(double));

    for (int i = 0; i < N + 1; i++) {
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "sl", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "su", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "lam", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "t", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "z", buffer);
        if (i < N) {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "pi", buffer);
            ocp_nlp_set(nlp_config, nlp_solver, i, "xdot_guess", buffer);
            ocp_nlp_set(nlp_config, nlp_solver, i, "z_guess", buffer);
        }
    }

    free(buffer);
    return 0;
}

int pendulum_ode_acados_update_params(pendulum_ode_solver_capsule* capsule, int stage, double* p, int np)
{
    int solver_status = 0;

    int casadi_np = 0;
    if (casadi_np != np) {
        printf("acados_update_params: trying to set %i parameters for external functions."
               " External function has %i parameters. Exiting.\n",
            np, casadi_np);
        exit(1);
    }

    return solver_status;
}

int pendulum_ode_acados_update_params_sparse(
    pendulum_ode_solver_capsule* capsule, int stage, int* idx, double* p, int n_update)
{
    int solver_status = 0;

    int casadi_np = 0;
    if (casadi_np < n_update) {
        printf("pendulum_ode_acados_update_params_sparse: trying to set %d parameters for external functions."
               " External function has %d parameters. Exiting.\n",
            n_update, casadi_np);
        exit(1);
    }
    // for (int i = 0; i < n_update; i++)
    // {
    //     if (idx[i] > casadi_np) {
    //         printf("pendulum_ode_acados_update_params_sparse: attempt to set parameters with index %d, while"
    //             " external functions only has %d parameters. Exiting.\n", idx[i], casadi_np);
    //         exit(1);
    //     }
    //     printf("param %d value %e\n", idx[i], p[i]);
    // }

    return 0;
}

int pendulum_ode_acados_solve(pendulum_ode_solver_capsule* capsule)
{
    // solve NLP
    int solver_status = ocp_nlp_solve(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}

int pendulum_ode_acados_free(pendulum_ode_solver_capsule* capsule)
{
    // before destroying, keep some info
    const int N = capsule->nlp_solver_plan->N;
    // free memory
    ocp_nlp_solver_opts_destroy(capsule->nlp_opts);
    ocp_nlp_in_destroy(capsule->nlp_in);
    ocp_nlp_out_destroy(capsule->nlp_out);
    ocp_nlp_out_destroy(capsule->sens_out);
    ocp_nlp_solver_destroy(capsule->nlp_solver);
    ocp_nlp_dims_destroy(capsule->nlp_dims);
    ocp_nlp_config_destroy(capsule->nlp_config);
    ocp_nlp_plan_destroy(capsule->nlp_solver_plan);

    /* free external function */
    // dynamics
    for (int i = 0; i < N; i++) {
        external_function_param_casadi_free(&capsule->impl_dae_fun[i]);
        external_function_param_casadi_free(&capsule->impl_dae_fun_jac_x_xdot_z[i]);
        external_function_param_casadi_free(&capsule->impl_dae_jac_x_xdot_u_z[i]);
    }
    free(capsule->impl_dae_fun);
    free(capsule->impl_dae_fun_jac_x_xdot_z);
    free(capsule->impl_dae_jac_x_xdot_u_z);

    // cost
    external_function_param_casadi_free(&capsule->ext_cost_0_fun);
    external_function_param_casadi_free(&capsule->ext_cost_0_fun_jac);
    external_function_param_casadi_free(&capsule->ext_cost_0_fun_jac_hess);
    for (int i = 0; i < N - 1; i++) {
        external_function_param_casadi_free(&capsule->ext_cost_fun[i]);
        external_function_param_casadi_free(&capsule->ext_cost_fun_jac[i]);
        external_function_param_casadi_free(&capsule->ext_cost_fun_jac_hess[i]);
    }
    free(capsule->ext_cost_fun);
    free(capsule->ext_cost_fun_jac);
    free(capsule->ext_cost_fun_jac_hess);
    external_function_param_casadi_free(&capsule->ext_cost_e_fun);
    external_function_param_casadi_free(&capsule->ext_cost_e_fun_jac);
    external_function_param_casadi_free(&capsule->ext_cost_e_fun_jac_hess);

    // constraints

    return 0;
}

ocp_nlp_in* pendulum_ode_acados_get_nlp_in(pendulum_ode_solver_capsule* capsule)
{
    return capsule->nlp_in;
}
ocp_nlp_out* pendulum_ode_acados_get_nlp_out(pendulum_ode_solver_capsule* capsule)
{
    return capsule->nlp_out;
}
ocp_nlp_out* pendulum_ode_acados_get_sens_out(pendulum_ode_solver_capsule* capsule)
{
    return capsule->sens_out;
}
ocp_nlp_solver* pendulum_ode_acados_get_nlp_solver(pendulum_ode_solver_capsule* capsule)
{
    return capsule->nlp_solver;
}
ocp_nlp_config* pendulum_ode_acados_get_nlp_config(pendulum_ode_solver_capsule* capsule)
{
    return capsule->nlp_config;
}
void* pendulum_ode_acados_get_nlp_opts(pendulum_ode_solver_capsule* capsule)
{
    return capsule->nlp_opts;
}
ocp_nlp_dims* pendulum_ode_acados_get_nlp_dims(pendulum_ode_solver_capsule* capsule)
{
    return capsule->nlp_dims;
}
ocp_nlp_plan_t* pendulum_ode_acados_get_nlp_plan(pendulum_ode_solver_capsule* capsule)
{
    return capsule->nlp_solver_plan;
}

void pendulum_ode_acados_print_stats(pendulum_ode_solver_capsule* capsule)
{
    int sqp_iter, stat_m, stat_n, tmp_int;
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "sqp_iter", &sqp_iter);
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "stat_n", &stat_n);
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "stat_m", &stat_m);

    double stat[12000];
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "statistics", stat);

    int nrow = sqp_iter + 1 < stat_m ? sqp_iter + 1 : stat_m;

    printf("iter\tres_stat\tres_eq\t\tres_ineq\tres_comp\tqp_stat\tqp_iter\talpha");
    if (stat_n > 8)
        printf("\t\tqp_res_stat\tqp_res_eq\tqp_res_ineq\tqp_res_comp");
    printf("\n");

    for (int i = 0; i < nrow; i++) {
        for (int j = 0; j < stat_n + 1; j++) {
            if (j == 0 || j == 5 || j == 6) {
                tmp_int = (int)stat[i + j * nrow];
                printf("%d\t", tmp_int);
            } else {
                printf("%e\t", stat[i + j * nrow]);
            }
        }
        printf("\n");
    }
}
