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
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

bool ModelPredictiveController::readReferenceFromFile(const char* fileName, vector<vector<double>>& reference) {
    // open file
    fstream file;
    file.open(fileName,ios::in);

    vector<double> lineData;
    string line;
    double value;

    if(file.is_open()) {
        // Skip the first (header) line
        getline(file, line);

        // Get all reference values
        while (getline(file, line)) {

            lineData.clear();

            stringstream ss(line);

            while (ss >> value) {
                lineData.push_back(value);
            }
            reference.push_back(lineData);
        }
        file.close();
    } else {
        return false;
    }
    return true;
}

void ModelPredictiveController::setReference(vector<vector<double>> reference) {
    for(int i = 0; i < ACADO_N; i++) {
        for(int j = 0; j < ACADO_NY; j++) {
            acadoVariables.y[i*ACADO_NY+j] = reference[i][j];
        }
    }
    for(int j = 0; j < ACADO_NYN; j++) {
        acadoVariables.yN[j] = reference[ACADO_N][j];
    }
}

void ModelPredictiveController::scrollReference(vector<vector<double>>& reference) {
    reference.erase(reference.begin());
    reference.push_back(reference[0]);
}

void ModelPredictiveController::init() {
  // Reset all solver memory
  memset(&acadoWorkspace, 0, sizeof(acadoWorkspace));
  memset(&acadoVariables, 0, sizeof(acadoVariables));

  // Read reference data
  vector<vector<double>> reference;
  if (ModelPredictiveController::readReferenceFromFile("references/sin.csv", reference) == false) {
      cout << "Cannot read reference" << endl;
  }

  // Initialize the solver
  acado_initializeSolver();

  // Prepare a consistent initial guess
  for (int i = 0; i < ACADO_N + 1; i++) {
    acadoVariables.x[i * ACADO_NX + 0] = 0; // theta
    acadoVariables.x[i * ACADO_NX + 1] = 0; // dtheta
  }

  for (int i = 0; i < ACADO_N; i++) {
      acadoVariables.u[i * ACADO_NU] = 0; // T
  }

  // Set reference and scroll one step up
  ModelPredictiveController::setReference(reference);
  ModelPredictiveController::scrollReference(reference);

  // Current state feedback
  ModelPredictiveController::setInitialState(x0);

  // Warm-up the solver
  acado_preparationStep();

}

void ModelPredictiveController::setInitialState(vector<double> x0) {
  for (int i = 0; i < ACADO_NX; ++i) {
    acadoVariables.x0[i] = x0[i];
  }
}

void ModelPredictiveController::controller() {

  // Set initial state
  setInitialState(x0);

  // Set reference
  ModelPredictiveController::setReference(reference);
  ModelPredictiveController::scrollReference(reference);

  // preparation step
  acado_preparationStep();

  // feedback step
  acado_feedbackStep( );
  u = acadoVariables.u[0];

  // Shift states and control and prepare for the next iteration
  acado_shiftStates(2, 0, 0);
  acado_shiftControls( 0 );

}

