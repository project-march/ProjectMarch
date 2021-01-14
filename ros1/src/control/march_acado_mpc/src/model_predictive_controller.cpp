#define _USE_MATH_DEFINES

#include "model_predictive_controller.hpp"
#include "acado_common.h"

#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;

// Global variables used by the solver
ACADOvariables acadoVariables = {};
ACADOworkspace acadoWorkspace = {};

bool ModelPredictiveController::readReferenceFromFile(const char* filename, vector<vector<double>>& data) {
    ifstream file(filename);
    string line;

    if (file.is_open()) {
        while (getline(file,line))
        {
            stringstream linestream(line);
            vector<double> linedata;
            double number;

            while (linestream >> number)
            {
                linedata.push_back(number);
            }
            data.push_back(linedata);
        }
        file.close();
    }
    else
        return false;

    return true;
}

void ModelPredictiveController::setReference(vector<vector<double>>& reference, int iter) {
    for (int i = 0; i < ACADO_N; ++i) {
        acadoVariables.y[i * ACADO_NY + 0] = reference[iter + i][0]; // theta
        acadoVariables.y[i * ACADO_NY + 1] = reference[iter + i][1]; // dtheta
        acadoVariables.y[i * ACADO_NY + 2] = reference[iter + i][2]; // T
    }

    acadoVariables.yN[0] = reference[iter + ACADO_N][0]; // theta
    acadoVariables.yN[1] = reference[iter + ACADO_N][1]; // dtheta
    acadoVariables.yN[2] = reference[iter + ACADO_N][2]; // T
}


void ModelPredictiveController::init() {

  // Read reference data
  if (ModelPredictiveController::readReferenceFromFile("../references/sin.txt", reference) == false) {
      cout << "Cannot read reference" << endl;
  }

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
//  ModelPredictiveController::setReference(reference, iter);

//  for (int i = 0; i < ACADO_N; ++i) {
//    acadoVariables.y[i * ACADO_NY + 0] = theta_ref; // theta
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

  // Set initial state
  setInitialState(x0);

  // Set reference
  ModelPredictiveController::setReference(reference, iter);

  // preparation step
  acado_preparationStep();

  // feedback step
  acado_feedbackStep();
  u = acadoVariables.u[0];

  // Shift states and control and prepare for the next iteration
  acado_shiftStates(2, 0, 0);
  acado_shiftControls(0);

  iter++;

}

