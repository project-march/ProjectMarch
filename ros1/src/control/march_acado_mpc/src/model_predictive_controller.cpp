#define _USE_MATH_DEFINES

#include "model_predictive_controller.hpp"
#include "acado_common.h"

#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

// Global variables used by the solver
ACADOvariables acadoVariables = {};
ACADOworkspace acadoWorkspace = {};

ModelPredictiveController::ModelPredictiveController(std::vector<std::vector<float>> Q)
    : Q_(Q)
{
}

void ModelPredictiveController::init()
{
  // Initialize the solver
  acado_initializeSolver();

  // Initialise the states vector
  for (int i = 0; i < ACADO_N + 1; ++i) {
    acadoVariables.x[i * ACADO_NX] = 0; // theta
    acadoVariables.x[i * ACADO_NX + 1] = 0; // dtheta
  }

  // Initialise the control input vector
  for (int i = 0; i < ACADO_N; i++) {
    acadoVariables.u[i * ACADO_NU] = 0; // T
  }

  // Set angle step reference value
  double theta_ref = 90*(M_PI/180);

  // Prepare references (step reference)
  for (int i = 0; i < ACADO_N; ++i) {
    acadoVariables.y[i * ACADO_NY] = theta_ref; // theta
    acadoVariables.y[i * ACADO_NY + 1] = 0;         // dtheta
    acadoVariables.y[i * ACADO_NY + 2] = 0;         // T
  }

  acadoVariables.yN[0] = theta_ref; // theta
  acadoVariables.yN[1] = 0;         // dtheta
  acadoVariables.yN[2] = 0;         // T

  // Current state feedback
  setInitialState(x0);

  // Warm-up the solver
  acado_preparationStep();

  assignWeightingMatrix(Q_);

}

void ModelPredictiveController::setInitialState(vector<double> x0) {
  for (int i = 0; i < ACADO_NX; ++i) {
    acadoVariables.x0[i] = x0[i];
  }
}

void ModelPredictiveController::assignWeightingMatrix(std::vector<float> Q) {

    double ACADO_NW = sizeof(acadoVariables.W)/sizeof(acadoVariables.W[0]);
    double ACADO_NWN = sizeof(acadoVariables.WN)/sizeof(acadoVariables.WN[0]);

    std::cout << ACADO_NW << ", " << ACADO_NWN << std::endl;

    int nrows = Q.size();
    int ncols = Q[0].size();

    // set W matrix with Q matrix
    for(int i=0; i < nrows; i++) {
        for(int j=0; j < ncols; j++) {
            acadoVariables.W[i*ncols+j] = Q[i][j];
        }
    }

    // Set WN matrix with Q matrix
    for(int i=0; i < (nrows-1); i++) {
        for(int j=0; j < 2; j++) {
            acadoVariables.WN[i*(ncols-1) + j] = Q[i][j];
        }
    }

    for(int i=0; i < ACADO_NW; i++) {
        std::cout << acadoVariables.W[i] << std::endl;
    }

    for(int i=0; i < ACADO_NWN; i++) {
        std::cout << acadoVariables.WN[i] << std::endl;
    }

  // [Temporary] Has the function been executed?
  std::cout << "\033[4;32m" << __FUNCTION__ << "()\033[0m" << " has executed\n";
}

void ModelPredictiveController::calculateControlInput() {

  // Set initial speed
  setInitialState(x0);

  // preparation step
  acado_preparationStep();

  // feedback step
  acado_feedbackStep();
  u = acadoVariables.u[0];

  // Shift states and control and prepare for the next iteration
  acado_shiftStates(2, 0, 0);
  acado_shiftControls(0);

}

