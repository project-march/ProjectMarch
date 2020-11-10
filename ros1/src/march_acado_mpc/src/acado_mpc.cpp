
#include <iostream>
#include <string>
#include <vector>

#include "acado_mpc.h"
#include "acado_common.h"
#include "acado_auxiliary_functions.h"

using namespace std;

// Some convenient definitions
#define NX          ACADO_NX  // Number of differential state variables
#define NXA         ACADO_NXA // Number of algebraic variables
#define NU          ACADO_NU  // Number of control inputs
#define NOD         ACADO_NOD // Number of online data values

#define NY          ACADO_NY  // Number of measurements/references on nodes 0..N - 1
#define NYN         ACADO_NYN // Number of measurements/references on node N

#define N           ACADO_N   // Number of intervals in the horizon

#define NUM_STEPS   10        // Number of real-time iterations

// global variables used by the solver
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

// ModelPredictiveController constructor
ModelPredictiveController::ModelPredictiveController(vector<string> joint_list){

    // assign joint names to _joint_list
    _joint_list = joint_list;

    // print _joint_list to the console (debug purposes)
    for (unsigned int i = 0; i < _joint_list.size(); i++) {
        cout << _joint_list[i] << endl;
    }
}

// initialise the solver
void ModelPredictiveController::initSolver( )
{
    // Reset all solver memory
    memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
    memset(&acadoVariables, 0, sizeof( acadoVariables ));

    //
    // Initialize the solver
    //
    acado_initializeSolver();

    //
    // Prepare a consistent initial guess
    //
    for (int i = 0; i < N + 1; ++i)
    {
        acadoVariables.x[i * NX + 0] = 0;
        acadoVariables.x[i * NX + 1] = 0;
    }

    //
    // Prepare references
    //
    for (int i = 0; i < N; ++i)
    {
        acadoVariables.y[i * NY + 0] = 0; // x1
        acadoVariables.y[i * NY + 1] = 0; // x2
        acadoVariables.y[i * NY + 2] = 0; // u
    }

    acadoVariables.yN[ 0 ] = 0; // x1
    acadoVariables.yN[ 1 ] = 0; // x2
    acadoVariables.yN[ 2 ] = 0; // u


}

// set the initial states
void ModelPredictiveController::setInitState(vector<double> x0)
{
    for (int i = 0; i < NX; ++i)
    {
        acadoVariables.x0[i] = x0[i];
    }
}

// run a single feedback iteration
void ModelPredictiveController::controller(vector<double> x0)
{
    // return the States and Control sequences
    // return the iteration time

    // setup iteration timer
    real_t t_iter;

    acado_timer t;
    acado_tic( &t );

    //
    // Warm-up the solver
    //
    acado_preparationStep();

    // set initial state from measurements
    ModelPredictiveController::setInitState(x0);

    acado_feedbackStep( );
    t_iter = acado_toc( &t );

    cout << t_iter * 1e6 << " microseconds" << endl;

}

// print debugging information
void ModelPredictiveController::printDebug(const bool VERBOSE)
{
    if (VERBOSE)
    {
        cout << "\nDebug Information\n-----------------\n";
        cout << "x0 = [" << acadoVariables.x0[0] << ", " << acadoVariables.x0[1] << "]" << endl;
        acado_printDifferentialVariables();
        acado_printControlVariables();
    }
}
