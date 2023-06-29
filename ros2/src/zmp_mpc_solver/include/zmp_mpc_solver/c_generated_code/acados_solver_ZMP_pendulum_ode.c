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
#include "ZMP_pendulum_ode_constraints/ZMP_pendulum_ode_constraints.h"
#include "ZMP_pendulum_ode_cost/ZMP_pendulum_ode_cost.h"
#include "ZMP_pendulum_ode_model/ZMP_pendulum_ode_model.h"

#include "acados_solver_ZMP_pendulum_ode.h"

#define NX ZMP_PENDULUM_ODE_NX
#define NZ ZMP_PENDULUM_ODE_NZ
#define NU ZMP_PENDULUM_ODE_NU
#define NP ZMP_PENDULUM_ODE_NP
#define NBX ZMP_PENDULUM_ODE_NBX
#define NBX0 ZMP_PENDULUM_ODE_NBX0
#define NBU ZMP_PENDULUM_ODE_NBU
#define NSBX ZMP_PENDULUM_ODE_NSBX
#define NSBU ZMP_PENDULUM_ODE_NSBU
#define NSH ZMP_PENDULUM_ODE_NSH
#define NSG ZMP_PENDULUM_ODE_NSG
#define NSPHI ZMP_PENDULUM_ODE_NSPHI
#define NSHN ZMP_PENDULUM_ODE_NSHN
#define NSGN ZMP_PENDULUM_ODE_NSGN
#define NSPHIN ZMP_PENDULUM_ODE_NSPHIN
#define NSBXN ZMP_PENDULUM_ODE_NSBXN
#define NS ZMP_PENDULUM_ODE_NS
#define NSN ZMP_PENDULUM_ODE_NSN
#define NG ZMP_PENDULUM_ODE_NG
#define NBXN ZMP_PENDULUM_ODE_NBXN
#define NGN ZMP_PENDULUM_ODE_NGN
#define NY0 ZMP_PENDULUM_ODE_NY0
#define NY ZMP_PENDULUM_ODE_NY
#define NYN ZMP_PENDULUM_ODE_NYN
// #define N      ZMP_PENDULUM_ODE_N
#define NH ZMP_PENDULUM_ODE_NH
#define NPHI ZMP_PENDULUM_ODE_NPHI
#define NHN ZMP_PENDULUM_ODE_NHN
#define NPHIN ZMP_PENDULUM_ODE_NPHIN
#define NR ZMP_PENDULUM_ODE_NR

// ** solver data **

ZMP_pendulum_ode_solver_capsule* ZMP_pendulum_ode_acados_create_capsule(void)
{
    void* capsule_mem = malloc(sizeof(ZMP_pendulum_ode_solver_capsule));
    ZMP_pendulum_ode_solver_capsule* capsule = (ZMP_pendulum_ode_solver_capsule*)capsule_mem;

    return capsule;
}

int ZMP_pendulum_ode_acados_free_capsule(ZMP_pendulum_ode_solver_capsule* capsule)
{
    free(capsule);
    return 0;
}

int ZMP_pendulum_ode_acados_create(ZMP_pendulum_ode_solver_capsule* capsule)
{
    int N_shooting_intervals = ZMP_PENDULUM_ODE_N;
    double* new_time_steps = NULL; // NULL -> don't alter the code generated time-steps
    return ZMP_pendulum_ode_acados_create_with_discretization(capsule, N_shooting_intervals, new_time_steps);
}

int ZMP_pendulum_ode_acados_update_time_steps(ZMP_pendulum_ode_solver_capsule* capsule, int N, double* new_time_steps)
{
    if (N != capsule->nlp_solver_plan->N) {
        fprintf(stderr,
            "ZMP_pendulum_ode_acados_update_time_steps: given number of time steps (= %d) "
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
 * Internal function for ZMP_pendulum_ode_acados_create: step 1
 */
void ZMP_pendulum_ode_acados_create_1_set_plan(ocp_nlp_plan_t* nlp_solver_plan, const int N)
{
    assert(N == nlp_solver_plan->N);

    /************************************************
     *  plan
     ************************************************/
    nlp_solver_plan->nlp_solver = SQP;

    nlp_solver_plan->ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;

    nlp_solver_plan->nlp_cost[0] = EXTERNAL;
    for (int i = 1; i < N; i++)
        nlp_solver_plan->nlp_cost[i] = EXTERNAL;

    nlp_solver_plan->nlp_cost[N] = EXTERNAL;

    for (int i = 0; i < N; i++) {
        nlp_solver_plan->nlp_dynamics[i] = CONTINUOUS_MODEL;
        nlp_solver_plan->sim_solver_plan[i].sim_solver = ERK;
    }

    for (int i = 0; i < N; i++) {
        nlp_solver_plan->nlp_constraints[i] = BGH;
    }
    nlp_solver_plan->nlp_constraints[N] = BGH;
}

/**
 * Internal function for ZMP_pendulum_ode_acados_create: step 2
 */
ocp_nlp_dims* ZMP_pendulum_ode_acados_create_2_create_and_set_dimensions(ZMP_pendulum_ode_solver_capsule* capsule)
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
    nbxe[0] = 12;
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
 * Internal function for ZMP_pendulum_ode_acados_create: step 3
 */
void ZMP_pendulum_ode_acados_create_3_create_and_set_functions(ZMP_pendulum_ode_solver_capsule* capsule)
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
        external_function_param_casadi_create(&capsule->__CAPSULE_FNC__, 5);                                           \
    } while (false)

    // explicit ode
    capsule->forw_vde_casadi = (external_function_param_casadi*)malloc(sizeof(external_function_param_casadi) * N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(forw_vde_casadi[i], ZMP_pendulum_ode_expl_vde_forw);
    }

    capsule->expl_ode_fun = (external_function_param_casadi*)malloc(sizeof(external_function_param_casadi) * N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(expl_ode_fun[i], ZMP_pendulum_ode_expl_ode_fun);
    }

    // external cost
    MAP_CASADI_FNC(ext_cost_0_fun, ZMP_pendulum_ode_cost_ext_cost_0_fun);

    // external cost
    MAP_CASADI_FNC(ext_cost_0_fun_jac, ZMP_pendulum_ode_cost_ext_cost_0_fun_jac);

    // external cost
    MAP_CASADI_FNC(ext_cost_0_fun_jac_hess, ZMP_pendulum_ode_cost_ext_cost_0_fun_jac_hess);
    // external cost
    capsule->ext_cost_fun = (external_function_param_casadi*)malloc(sizeof(external_function_param_casadi) * N);
    for (int i = 0; i < N - 1; i++) {
        MAP_CASADI_FNC(ext_cost_fun[i], ZMP_pendulum_ode_cost_ext_cost_fun);
    }

    capsule->ext_cost_fun_jac = (external_function_param_casadi*)malloc(sizeof(external_function_param_casadi) * N);
    for (int i = 0; i < N - 1; i++) {
        MAP_CASADI_FNC(ext_cost_fun_jac[i], ZMP_pendulum_ode_cost_ext_cost_fun_jac);
    }

    capsule->ext_cost_fun_jac_hess
        = (external_function_param_casadi*)malloc(sizeof(external_function_param_casadi) * N);
    for (int i = 0; i < N - 1; i++) {
        MAP_CASADI_FNC(ext_cost_fun_jac_hess[i], ZMP_pendulum_ode_cost_ext_cost_fun_jac_hess);
    }
    // external cost - function
    MAP_CASADI_FNC(ext_cost_e_fun, ZMP_pendulum_ode_cost_ext_cost_e_fun);

    // external cost - jacobian
    MAP_CASADI_FNC(ext_cost_e_fun_jac, ZMP_pendulum_ode_cost_ext_cost_e_fun_jac);

    // external cost - hessian
    MAP_CASADI_FNC(ext_cost_e_fun_jac_hess, ZMP_pendulum_ode_cost_ext_cost_e_fun_jac_hess);

#undef MAP_CASADI_FNC
}

/**
 * Internal function for ZMP_pendulum_ode_acados_create: step 4
 */
void ZMP_pendulum_ode_acados_create_4_set_default_parameters(ZMP_pendulum_ode_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    // initialize parameters to nominal value
    double* p = calloc(NP, sizeof(double));

    for (int i = 0; i <= N; i++) {
        ZMP_pendulum_ode_acados_update_params(capsule, i, p, NP);
    }
    free(p);
}

/**
 * Internal function for ZMP_pendulum_ode_acados_create: step 5
 */
void ZMP_pendulum_ode_acados_create_5_set_nlp_in(
    ZMP_pendulum_ode_solver_capsule* capsule, const int N, double* new_time_steps)
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
        ZMP_pendulum_ode_acados_update_time_steps(capsule, N, new_time_steps);
    } else { // all time_steps are identical
        double time_step = 0.00796812749003984;
        for (int i = 0; i < N; i++) {
            ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &time_step);
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &time_step);
        }
    }

    /**** Dynamics ****/
    for (int i = 0; i < N; i++) {
        ocp_nlp_dynamics_model_set(nlp_config, nlp_dims, nlp_in, i, "expl_vde_forw", &capsule->forw_vde_casadi[i]);
        ocp_nlp_dynamics_model_set(nlp_config, nlp_dims, nlp_in, i, "expl_ode_fun", &capsule->expl_ode_fun[i]);
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
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "ext_cost_fun", &capsule->ext_cost_e_fun);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "ext_cost_fun_jac", &capsule->ext_cost_e_fun_jac);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "ext_cost_fun_jac_hess", &capsule->ext_cost_e_fun_jac_hess);

    /**** Constraints ****/

    // bounds for initial stage
    // x0
    int* idxbx0 = malloc(NBX0 * sizeof(int));
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

    double* lubx0 = calloc(2 * NBX0, sizeof(double));
    double* lbx0 = lubx0;
    double* ubx0 = lubx0 + NBX0;
    // change only the non-zero elements:
    lbx0[3] = 0.165;
    ubx0[3] = 0.165;
    lbx0[5] = 0.165;
    ubx0[5] = 0.165;
    lbx0[9] = 0.33;
    ubx0[9] = 0.33;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);
    free(idxbx0);
    free(lubx0);
    // idxbxe_0
    int* idxbxe_0 = malloc(12 * sizeof(int));

    idxbxe_0[0] = 0;
    idxbxe_0[1] = 1;
    idxbxe_0[2] = 2;
    idxbxe_0[3] = 3;
    idxbxe_0[4] = 4;
    idxbxe_0[5] = 5;
    idxbxe_0[6] = 6;
    idxbxe_0[7] = 7;
    idxbxe_0[8] = 8;
    idxbxe_0[9] = 9;
    idxbxe_0[10] = 10;
    idxbxe_0[11] = 11;
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbxe", idxbxe_0);
    free(idxbxe_0);

    /* constraints that are the same for initial and intermediate */

    /* terminal constraints */
}

/**
 * Internal function for ZMP_pendulum_ode_acados_create: step 6
 */
void ZMP_pendulum_ode_acados_create_6_set_opts(ZMP_pendulum_ode_solver_capsule* capsule)
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
    int sim_method_num_stages = 1;
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
    int qp_solver_cond_N;

    // NOTE: there is no condensing happening here!
    qp_solver_cond_N = N;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_cond_N", &qp_solver_cond_N);

    int nlp_solver_ext_qp_res = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "ext_qp_res", &nlp_solver_ext_qp_res);
    // set HPIPM mode: should be done before setting other QP solver options
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_hpipm_mode", "BALANCE");

    // set SQP specific options
    double nlp_solver_tol_stat = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_stat", &nlp_solver_tol_stat);

    double nlp_solver_tol_eq = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_eq", &nlp_solver_tol_eq);

    double nlp_solver_tol_ineq = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_ineq", &nlp_solver_tol_ineq);

    double nlp_solver_tol_comp = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_comp", &nlp_solver_tol_comp);

    int nlp_solver_max_iter = 100;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "max_iter", &nlp_solver_max_iter);

    int initialize_t_slacks = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "initialize_t_slacks", &initialize_t_slacks);

    int qp_solver_iter_max = 50;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_iter_max", &qp_solver_iter_max);

    int print_level = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "print_level", &print_level);
    int qp_solver_cond_ric_alg = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_cond_ric_alg", &qp_solver_cond_ric_alg);

    int qp_solver_ric_alg = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_ric_alg", &qp_solver_ric_alg);

    int ext_cost_num_hess = 0;
    for (int i = 0; i < N; i++) {
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "cost_numerical_hessian", &ext_cost_num_hess);
    }
    ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, N, "cost_numerical_hessian", &ext_cost_num_hess);
}

/**
 * Internal function for ZMP_pendulum_ode_acados_create: step 7
 */
void ZMP_pendulum_ode_acados_create_7_set_nlp_out(ZMP_pendulum_ode_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;

    // initialize primal solution
    double* xu0 = calloc(NX + NU, sizeof(double));
    double* x0 = xu0;

    // initialize with x0

    x0[3] = 0.165;
    x0[5] = 0.165;
    x0[9] = 0.33;

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
 * Internal function for ZMP_pendulum_ode_acados_create: step 8
 */
// void ZMP_pendulum_ode_acados_create_8_create_solver(ZMP_pendulum_ode_solver_capsule* capsule)
//{
//    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts);
//}

/**
 * Internal function for ZMP_pendulum_ode_acados_create: step 9
 */
int ZMP_pendulum_ode_acados_create_9_precompute(ZMP_pendulum_ode_solver_capsule* capsule)
{
    int status = ocp_nlp_precompute(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    if (status != ACADOS_SUCCESS) {
        printf("\nocp_nlp_precompute failed!\n\n");
        exit(1);
    }

    return status;
}

int ZMP_pendulum_ode_acados_create_with_discretization(
    ZMP_pendulum_ode_solver_capsule* capsule, int N, double* new_time_steps)
{
    // If N does not match the number of shooting intervals used for code generation, new_time_steps must be given.
    if (N != ZMP_PENDULUM_ODE_N && !new_time_steps) {
        fprintf(stderr,
            "ZMP_pendulum_ode_acados_create_with_discretization: new_time_steps is NULL "
            "but the number of shooting intervals (= %d) differs from the number of "
            "shooting intervals (= %d) during code generation! Please provide a new vector of time_stamps!\n",
            N, ZMP_PENDULUM_ODE_N);
        return 1;
    }

    // number of expected runtime parameters
    capsule->nlp_np = NP;

    // 1) create and set nlp_solver_plan; create nlp_config
    capsule->nlp_solver_plan = ocp_nlp_plan_create(N);
    ZMP_pendulum_ode_acados_create_1_set_plan(capsule->nlp_solver_plan, N);
    capsule->nlp_config = ocp_nlp_config_create(*capsule->nlp_solver_plan);

    // 3) create and set dimensions
    capsule->nlp_dims = ZMP_pendulum_ode_acados_create_2_create_and_set_dimensions(capsule);
    ZMP_pendulum_ode_acados_create_3_create_and_set_functions(capsule);

    // 4) set default parameters in functions
    ZMP_pendulum_ode_acados_create_4_set_default_parameters(capsule);

    // 5) create and set nlp_in
    capsule->nlp_in = ocp_nlp_in_create(capsule->nlp_config, capsule->nlp_dims);
    ZMP_pendulum_ode_acados_create_5_set_nlp_in(capsule, N, new_time_steps);

    // 6) create and set nlp_opts
    capsule->nlp_opts = ocp_nlp_solver_opts_create(capsule->nlp_config, capsule->nlp_dims);
    ZMP_pendulum_ode_acados_create_6_set_opts(capsule);

    // 7) create and set nlp_out
    // 7.1) nlp_out
    capsule->nlp_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    // 7.2) sens_out
    capsule->sens_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    ZMP_pendulum_ode_acados_create_7_set_nlp_out(capsule);

    // 8) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts);
    // ZMP_pendulum_ode_acados_create_8_create_solver(capsule);

    // 9) do precomputations
    int status = ZMP_pendulum_ode_acados_create_9_precompute(capsule);

    return status;
}

/**
 * This function is for updating an already initialized solver with a different number of qp_cond_N. It is useful for
 * code reuse after code export.
 */
int ZMP_pendulum_ode_acados_update_qp_solver_cond_N(ZMP_pendulum_ode_solver_capsule* capsule, int qp_solver_cond_N)
{
    // 1) destroy solver
    ocp_nlp_solver_destroy(capsule->nlp_solver);

    // 2) set new value for "qp_cond_N"
    const int N = capsule->nlp_solver_plan->N;
    if (qp_solver_cond_N > N)
        printf("Warning: qp_solver_cond_N = %d > N = %d\n", qp_solver_cond_N, N);
    ocp_nlp_solver_opts_set(capsule->nlp_config, capsule->nlp_opts, "qp_cond_N", &qp_solver_cond_N);

    // 3) continue with the remaining steps from ZMP_pendulum_ode_acados_create_with_discretization(...):
    // -> 8) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts);

    // -> 9) do precomputations
    int status = ZMP_pendulum_ode_acados_create_9_precompute(capsule);
    return status;
}

int ZMP_pendulum_ode_acados_reset(ZMP_pendulum_ode_solver_capsule* capsule, int reset_qp_solver_mem)
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
        }
    }
    // get qp_status: if NaN -> reset memory
    int qp_status;
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "qp_status", &qp_status);
    if (reset_qp_solver_mem || (qp_status == 3)) {
        // printf("\nin reset qp_status %d -> resetting QP memory\n", qp_status);
        ocp_nlp_solver_reset_qp_memory(nlp_solver, nlp_in, nlp_out);
    }

    free(buffer);
    return 0;
}

int ZMP_pendulum_ode_acados_update_params(ZMP_pendulum_ode_solver_capsule* capsule, int stage, double* p, int np)
{
    int solver_status = 0;

    int casadi_np = 5;
    if (casadi_np != np) {
        printf("acados_update_params: trying to set %i parameters for external functions."
               " External function has %i parameters. Exiting.\n",
            np, casadi_np);
        exit(1);
    }

    const int N = capsule->nlp_solver_plan->N;
    if (stage < N && stage >= 0) {
        capsule->forw_vde_casadi[stage].set_param(capsule->forw_vde_casadi + stage, p);
        capsule->expl_ode_fun[stage].set_param(capsule->expl_ode_fun + stage, p);

        // constraints

        // cost
        if (stage == 0) {
            capsule->ext_cost_0_fun.set_param(&capsule->ext_cost_0_fun, p);
            capsule->ext_cost_0_fun_jac.set_param(&capsule->ext_cost_0_fun_jac, p);
            capsule->ext_cost_0_fun_jac_hess.set_param(&capsule->ext_cost_0_fun_jac_hess, p);

        } else // 0 < stage < N
        {
            capsule->ext_cost_fun[stage - 1].set_param(capsule->ext_cost_fun + stage - 1, p);
            capsule->ext_cost_fun_jac[stage - 1].set_param(capsule->ext_cost_fun_jac + stage - 1, p);
            capsule->ext_cost_fun_jac_hess[stage - 1].set_param(capsule->ext_cost_fun_jac_hess + stage - 1, p);
        }
    }

    else // stage == N
    {
        // terminal shooting node has no dynamics
        // cost
        capsule->ext_cost_e_fun.set_param(&capsule->ext_cost_e_fun, p);
        capsule->ext_cost_e_fun_jac.set_param(&capsule->ext_cost_e_fun_jac, p);
        capsule->ext_cost_e_fun_jac_hess.set_param(&capsule->ext_cost_e_fun_jac_hess, p);

        // constraints
    }

    return solver_status;
}

int ZMP_pendulum_ode_acados_update_params_sparse(
    ZMP_pendulum_ode_solver_capsule* capsule, int stage, int* idx, double* p, int n_update)
{
    int solver_status = 0;

    int casadi_np = 5;
    if (casadi_np < n_update) {
        printf("ZMP_pendulum_ode_acados_update_params_sparse: trying to set %d parameters for external functions."
               " External function has %d parameters. Exiting.\n",
            n_update, casadi_np);
        exit(1);
    }
    // for (int i = 0; i < n_update; i++)
    // {
    //     if (idx[i] > casadi_np) {
    //         printf("ZMP_pendulum_ode_acados_update_params_sparse: attempt to set parameters with index %d, while"
    //             " external functions only has %d parameters. Exiting.\n", idx[i], casadi_np);
    //         exit(1);
    //     }
    //     printf("param %d value %e\n", idx[i], p[i]);
    // }
    const int N = capsule->nlp_solver_plan->N;
    if (stage < N && stage >= 0) {
        capsule->forw_vde_casadi[stage].set_param_sparse(capsule->forw_vde_casadi + stage, n_update, idx, p);
        capsule->expl_ode_fun[stage].set_param_sparse(capsule->expl_ode_fun + stage, n_update, idx, p);

        // constraints

        // cost
        if (stage == 0) {
            capsule->ext_cost_0_fun.set_param_sparse(&capsule->ext_cost_0_fun, n_update, idx, p);
            capsule->ext_cost_0_fun_jac.set_param_sparse(&capsule->ext_cost_0_fun_jac, n_update, idx, p);
            capsule->ext_cost_0_fun_jac_hess.set_param_sparse(&capsule->ext_cost_0_fun_jac_hess, n_update, idx, p);

        } else // 0 < stage < N
        {
            capsule->ext_cost_fun[stage - 1].set_param_sparse(capsule->ext_cost_fun + stage - 1, n_update, idx, p);
            capsule->ext_cost_fun_jac[stage - 1].set_param_sparse(
                capsule->ext_cost_fun_jac + stage - 1, n_update, idx, p);
            capsule->ext_cost_fun_jac_hess[stage - 1].set_param_sparse(
                capsule->ext_cost_fun_jac_hess + stage - 1, n_update, idx, p);
        }
    }

    else // stage == N
    {
        // terminal shooting node has no dynamics
        // cost
        capsule->ext_cost_e_fun.set_param_sparse(&capsule->ext_cost_e_fun, n_update, idx, p);
        capsule->ext_cost_e_fun_jac.set_param_sparse(&capsule->ext_cost_e_fun_jac, n_update, idx, p);
        capsule->ext_cost_e_fun_jac_hess.set_param_sparse(&capsule->ext_cost_e_fun_jac_hess, n_update, idx, p);

        // constraints
    }

    return 0;
}

int ZMP_pendulum_ode_acados_solve(ZMP_pendulum_ode_solver_capsule* capsule)
{
    // solve NLP
    int solver_status = ocp_nlp_solve(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}

int ZMP_pendulum_ode_acados_free(ZMP_pendulum_ode_solver_capsule* capsule)
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
        external_function_param_casadi_free(&capsule->forw_vde_casadi[i]);
        external_function_param_casadi_free(&capsule->expl_ode_fun[i]);
    }
    free(capsule->forw_vde_casadi);
    free(capsule->expl_ode_fun);

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

void ZMP_pendulum_ode_acados_print_stats(ZMP_pendulum_ode_solver_capsule* capsule)
{
    int sqp_iter, stat_m, stat_n, tmp_int;
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "sqp_iter", &sqp_iter);
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "stat_n", &stat_n);
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "stat_m", &stat_m);

    double stat[1200];
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

int ZMP_pendulum_ode_acados_custom_update(ZMP_pendulum_ode_solver_capsule* capsule, double* data, int data_len)
{
    printf("\ndummy function that can be called in between solver calls to update parameters or numerical data "
           "efficiently in C.\n");
    printf("nothing set yet..\n");
    return 1;
}

ocp_nlp_in* ZMP_pendulum_ode_acados_get_nlp_in(ZMP_pendulum_ode_solver_capsule* capsule)
{
    return capsule->nlp_in;
}
ocp_nlp_out* ZMP_pendulum_ode_acados_get_nlp_out(ZMP_pendulum_ode_solver_capsule* capsule)
{
    return capsule->nlp_out;
}
ocp_nlp_out* ZMP_pendulum_ode_acados_get_sens_out(ZMP_pendulum_ode_solver_capsule* capsule)
{
    return capsule->sens_out;
}
ocp_nlp_solver* ZMP_pendulum_ode_acados_get_nlp_solver(ZMP_pendulum_ode_solver_capsule* capsule)
{
    return capsule->nlp_solver;
}
ocp_nlp_config* ZMP_pendulum_ode_acados_get_nlp_config(ZMP_pendulum_ode_solver_capsule* capsule)
{
    return capsule->nlp_config;
}
void* ZMP_pendulum_ode_acados_get_nlp_opts(ZMP_pendulum_ode_solver_capsule* capsule)
{
    return capsule->nlp_opts;
}
ocp_nlp_dims* ZMP_pendulum_ode_acados_get_nlp_dims(ZMP_pendulum_ode_solver_capsule* capsule)
{
    return capsule->nlp_dims;
}
ocp_nlp_plan_t* ZMP_pendulum_ode_acados_get_nlp_plan(ZMP_pendulum_ode_solver_capsule* capsule)
{
    return capsule->nlp_solver_plan;
}
