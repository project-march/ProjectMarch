#define _USE_MATH_DEFINES

#include "model_predictive_controller.hpp"
#include "acado_common.h"

#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

// Global variables used by the solver
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

void ModelPredictiveController::sinRef(vector<vector<double>>& reference, double freq, double amp, int N, double dt) {

  // calculate the amount of time steps required for one period
  double T_N = ceil(1/(freq*dt));

  // multiply the required time steps with the amount of required periods to have at least N reference values
  T_N = T_N*ceil(N/T_N)+1;

  vector<double> line;
  for (int j = 0; j < T_N; j++) {
    // clear the line vector
    line.clear();

    // fill the line vector with the three required references values
    line.push_back(amp*sin((freq*2*M_PI)*(j*dt))); // THETA
    line.push_back(amp*cos((freq*2*M_PI)*(j*dt))); // DTHETA
    line.push_back(0.0); // T

    // add the temp vector to the reference
    reference.push_back(line);
  }

}

void ModelPredictiveController::stepRef(vector<vector<double>>& ref, double amp, int N, double dt) {

    vector<double> line;
    for (int j = 0; j < N+1; j++) {
        // clear the line vector
        line.clear();

        // fill the line vector with the three required references values
        line.push_back(amp); // THETA
        line.push_back(0.0); // DTHETA
        line.push_back(0.0); // T

        // add the temp vector to the reference
        ref.push_back(line);
    }

}

void ModelPredictiveController::scrollReference(vector<vector<double>>& reference) {
    reference.erase(reference.begin());
    reference.push_back(reference[0]);
}

void ModelPredictiveController::setReference(vector<vector<double>> reference) {
    for(int i = 0; i < ACADO_N; i++) {
        for(int j = 0; j < ACADO_NY; j++) {
            acadoVariables.y[i * ACADO_NY + j] = reference[i][j];
        }
    }
    for(int j = 0; j < ACADO_NYN; j++) {
        acadoVariables.yN[j] = reference[ACADO_N][j];
    }
}


void ModelPredictiveController::init() {
  // Reset all solver memory
  memset(&acadoWorkspace, 0, sizeof(acadoWorkspace));
  memset(&acadoVariables, 0, sizeof(acadoVariables));

  // Initialize the solver
  acado_initializeSolver();

  // Prepare a consistent initial guess
  for (int i = 0; i < ACADO_N + 1; ++i) {
    acadoVariables.x[i * ACADO_NX] = 0; // theta
    acadoVariables.x[i * ACADO_NX + 1] = 0; // dtheta
  }

//  vector<vector<double>> reference;
  sinRef(reference, 0.1, 0.25, ACADO_N, 0.04);
//  stepRef(reference, 0.0, ACADO_N+1, 0.04);
  bool repeat_reference = true;

  // Set the reference
  setReference(reference);

//  double theta_ref = 90*(M_PI/180);
//
//  // Prepare references (step reference)
//  for (int i = 0; i < ACADO_N; ++i) {
//    acadoVariables.y[i * ACADO_NY] = theta_ref; // theta
//    acadoVariables.y[i * ACADO_NY + 1] = 0;         // dtheta
//    acadoVariables.y[i * ACADO_NY + 2] = 0;         // T
//  }
//
//  acadoVariables.yN[0] = theta_ref; // theta
//  acadoVariables.yN[1] = 0;         // dtheta
//  acadoVariables.yN[2] = 0;         // T

  // Current state feedback
  setInitialState(x0);

  // Warm-up the solver
  acado_preparationStep();

}

void ModelPredictiveController::setInitialState(vector<double> x0) {
  for (int i = 0; i < ACADO_NX; ++i) {
    acadoVariables.x0[i] = x0[i];
  }
}

void ModelPredictiveController::calculateControlInput() {

  // Set initial speed
  setInitialState(x0);

  // preparation step
  setReference(reference);
  acado_preparationStep();

  // feedback step
  acado_feedbackStep();
  u = acadoVariables.u[0];

  // Shift states and control and prepare for the next iteration
  acado_shiftStates(2, 0, 0);
  acado_shiftControls(0);

  // Scroll the reference vector
  if(true) {
      scrollReference(reference);
  }

}
