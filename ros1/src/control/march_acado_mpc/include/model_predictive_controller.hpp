#ifndef MARCH_MODELPREDICTIVECONTROLLER_H
#define MARCH_MODELPREDICTIVECONTROLLER_H

#include <vector>
#include "acado_common.h"

class ModelPredictiveController {

public:

/**
 * \brief Initalise the solver
 */
  void initSolver();

  /**
   * \brief Set the initial state
   * @param x0 - initial state
   */
  void setInitialState(std::vector<double> x0);


private:

  // Some convenient definitions
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

};

#endif  // MARCH_MODELPREDICTIVECONTROLLER_H
