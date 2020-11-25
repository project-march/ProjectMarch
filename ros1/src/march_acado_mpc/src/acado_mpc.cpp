
#include <iostream>
#include <string>
#include <vector>

#include "acado_mpc.h"
#include "acado_common.h"
#include "acado_auxiliary_functions.h"

//Some convenient definitions
#define NX          ACADO_NX  // Number of differential state variables
#define NXA         ACADO_NXA // Number of algebraic variables
#define NU          ACADO_NU  // Number of control inputs
#define NOD         ACADO_NOD // Number of online data values

#define NY          ACADO_NY  // Number of measurements/references on nodes 0..N - 1
#define NYN         ACADO_NYN // Number of measurements/references on node N

#define N           ACADO_N   // Number of intervals in the horizon

// global variables used by the solver
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

// ModelPredictiveController constructor
ModelPredictiveController::ModelPredictiveController() { }

// initialise the solver
void ModelPredictiveController::init( )
{
    // Reset all solver memory
    memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
    memset(&acadoVariables, 0, sizeof( acadoVariables ));

    // Initialize the solver
    acado_initializeSolver();

    // Prepare a consistent initial guess
    for (int i = 0; i < N + 1; ++i)
    {
        acadoVariables.x[i * NX + 0] = 0; // x1
        acadoVariables.x[i * NX + 1] = 0; // x2
        acadoVariables.x[i * NX + 2] = 0; // u
    }

    // Prepare references
    for (int i = 0; i < N; ++i)
    {
        acadoVariables.y[i * NY + 0] = 0; // x1
        acadoVariables.y[i * NY + 1] = 0; // x2
        acadoVariables.y[i * NY + 2] = 0; // du
    }

    acadoVariables.yN[ 0 ] = 1; // x1
    acadoVariables.yN[ 1 ] = 0; // x2
    acadoVariables.yN[ 2 ] = 0; // du

    std::cout << NX << std::endl;
}
