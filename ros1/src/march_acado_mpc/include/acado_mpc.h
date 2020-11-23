
#ifndef MARCH_ACADO_MPC_ACADO_MPC_H
#define MARCH_ACADO_MPC_ACADO_MPC_H

#include <string>
#include <vector>
#include "acado_auxiliary_functions.h"

using namespace std;

class ModelPredictiveController {

private:
    vector<string> _joint_list;

public:

    /**
     * \brief Constructor function
     */
    ModelPredictiveController(vector<string> joint_list);

    /**
     * \brief Initalise the solver
     */
    void initSolver( );

    /**
     * \brief Set the initial state
     */
    void setInitState(vector<double> x0);

    /**
     * \brief
     */
//    vector<float> getState();

    /**
     * \brief
     */
//    vector<float> getControl( );

    /**
     * \brief Run a single feedback iteration given the initial state
     */
    void controller(vector<double> x0);

    /**
     * \brief Print debug information
     */
    void printDebug(const bool VERBOSE);
};

#endif //MARCH_ACADO_MPC_ACADO_MPC_H
