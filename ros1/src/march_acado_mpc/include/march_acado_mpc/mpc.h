
#ifndef MARCH_ACADO_MPC_H
#define MARCH_ACADO_MPC_H

#include "../../src/mpc_codegen/acado_common.h"
#include "../../src/mpc_codegen/acado_auxiliary_functions.h"


/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   10        /* Number of real-time iterations. */

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

class ModelPredictiveController {

public:

    /**
     * \brief Init solver
     */
    void initSolver();

    /**
     * \brief Set initial state
     */
    void setInitState(double x0[NX]);

    /**
     * \brief Print debug information
     */
    void printDebug(const bool VERBOSE);

    /**
     * \brief Prepare the solver and run the feedback step
     */
    void controller(double x0[NX]);
};

#endif // MARCH_ACADO_MPC_H