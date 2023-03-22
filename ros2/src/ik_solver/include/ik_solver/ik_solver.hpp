//
// Created by Tessel & Jack March 6th 2023
//

#include <vector>
#include <iostream>
#include <memory>
#include "pinocchio/fwd.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

// TEST
// #include "pinocchio/parsers/sample-models.hpp"

#ifndef IK_SOLVER_JACOBIAN_GENERATOR_HPP
#define IK_SOLVER_JACOBIAN_GENERATOR_HPP

class IkSolver {
public: 
    IkSolver();
    void load_urdf_model(std::string);
    void set_joint_configuration();
    int set_jacobian();
    int get_model_joints();
    pinocchio::Data::Matrix6x get_model_jacobian();


private:
    pinocchio::Model m_model;
    pinocchio::Data m_model_data;

    Eigen::VectorXd m_joint_pos;
    Eigen::VectorXd m_joint_vel;
    Eigen::VectorXd m_joint_acc;

    pinocchio::Data::Matrix6x Jfoot;
};


#endif // IK_SOLVER_JACOBIAN_GENERATOR_HPP