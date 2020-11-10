
#ifndef MARCH_ACADO_MPC_ACADO_MPC_H
#define MARCH_ACADO_MPC_ACADO_MPC_H

#include "acado_common.h"
#include "acado_auxiliary_functions.h"
#include <string>
#include <iostream>
using namespace std;

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   10        /* Number of real-time iterations. */


class ModelPredictiveController {

private:

public:
    /**
     * \brief Constructure function
     */
    ModelPredictiveController();

};

#endif //MARCH_ACADO_MPC_ACADO_MPC_H
