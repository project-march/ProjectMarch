
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

#define NUM_STEPS   100        // Number of real-time iterations

// global variables used by the solver
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

// ModelPredictiveController constructor
ModelPredictiveController::ModelPredictiveController(vector<string> joint_list){

    // assign joint names to _joint_list
    _joint_list = joint_list;

}

// initialise the solver
void ModelPredictiveController::initSolver( )
{
    // Reset all solver memory
    memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
    memset(&acadoVariables, 0, sizeof( acadoVariables ));

    // Initialize the solver
    acado_initializeSolver();

    // Prepare a consistent initial guess
    for (int i = 0; i < N + 1; ++i)
    {
        acadoVariables.x[i * NX + 0] = 0;
        acadoVariables.x[i * NX + 1] = 0;
    }

    // Prepare references
    for (int i = 0; i < N; ++i)
    {
        acadoVariables.y[i * NY + 0] = 0; // x1
        acadoVariables.y[i * NY + 1] = 0; // x2
        acadoVariables.y[i * NY + 2] = 0; // u
    }

    acadoVariables.yN[ 0 ] = 1; // x1
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

//vector<float> ModelPredictiveController::getState()
//{
//    return (vector<float>)acadoVariables.x
//}

//vector<float> ModelPredictiveController::getControl( ) {
//    return acadoVariables.u
//}

// run a single feedback iteration
void ModelPredictiveController::controller(vector<double> x0)
{
    int iter, i;

    // set initial state from measurements
    ModelPredictiveController::setInitState(x0);

    // Warm-up the solver
    acado_preparationStep();

    for (iter = 0; iter < NUM_STEPS; iter++)
    {
        acado_feedbackStep( );

//        acado_printDifferentialVariables();
//		acado_printControlVariables();
        cout << acadoVariables.x[0] << ", " << acadoVariables.x[1] << endl;


		for (i = 0; i < NX; i++)
        {
		    acadoVariables.x0[i] = acadoVariables.x[NX + i];
        }

        // Shift states and controls
        acado_shiftStates(2, 0, 0);
        acado_shiftControls( 0 );

        acado_preparationStep();

    }

    // perform a single feedback step


}

// print debugging information
void ModelPredictiveController::printDebug(const bool VERBOSE)
{
    if (VERBOSE)
    {
        cout << "\nDebug Information\n-----------------\n";

        // print _joint_list to the console (debug purposes)
        cout << "Detected joints:\n[";
        for (unsigned int i = 0; i < _joint_list.size(); i++) {
            cout << " " <<_joint_list[i] << " ";
        }
        cout << "]\n";

        cout << "\nInitial State ";
        cout << "x0 = [" << acadoVariables.x0[0] << ", " << acadoVariables.x0[1] << "]" << endl;

        cout << "\nSolver output\n-------------";
        acado_printDifferentialVariables();
        acado_printControlVariables();
    }
}
