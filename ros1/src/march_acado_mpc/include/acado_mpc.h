
#ifndef MARCH_ACADO_MPC_ACADO_MPC_H
#define MARCH_ACADO_MPC_ACADO_MPC_H

#include <string>
#include <vector>
#include "acado_common.h"
#include "acado_auxiliary_functions.h"


class ModelPredictiveController {

public:

    int num_;

    /**
     * \brief Constructor function
     */
    ModelPredictiveController();

    /**
     * \brief Initalise the solver
     */
    void init( );

private:



};

#endif //MARCH_ACADO_MPC_ACADO_MPC_H
