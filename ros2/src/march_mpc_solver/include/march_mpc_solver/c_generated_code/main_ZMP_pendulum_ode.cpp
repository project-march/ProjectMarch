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
#include <array>
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/math.h"
#include "acados/utils/print.h"
#include "acados_c/external_function_interface.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_ZMP_pendulum_ode.h"

// blasfeo
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

// March
// #include "zmp_mpc_solver/zmp_mpc_solver.hpp"

#ifndef C_GENERATED_MPC
#define C_GENERATED_MPC

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
#define NH ZMP_PENDULUM_ODE_NH
#define NPHI ZMP_PENDULUM_ODE_NPHI
#define NHN ZMP_PENDULUM_ODE_NHN
#define NPHIN ZMP_PENDULUM_ODE_NPHIN
#define NR ZMP_PENDULUM_ODE_NR

#endif