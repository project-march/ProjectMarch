//
// Created by Sahand March 21st 2023
//

// Largely based on Nicola Scianca's work and code:
// https://ieeexplore.ieee.org/document/7803336

#include "ik_solver/utils.hpp"
#include "labrob_qpsolvers/qpsolvers.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/fwd.hpp"
#include "pinocchio/math/rpy.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

// TEST
// #include "pinocchio/parsers/sample-models.hpp"

#ifndef IK_SOLVER_JACOBIAN_GENERATOR_HPP
#define IK_SOLVER_JACOBIAN_GENERATOR_HPP

struct state {
    // Pose, in this case, is [POSITION, ANGLE]
    Eigen::Matrix<double, 6, 1> left_foot_pose = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> right_foot_pose = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 3, 1> com_pos = Eigen::Vector3d::Zero();

    Eigen::Matrix<double, 6, 1> left_foot_vel = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> right_foot_vel = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 3, 1> com_vel = Eigen::Vector3d::Zero();
};

class IkSolver {
public:
    IkSolver();
    void load_urdf_model(std::string);
    void set_joint_configuration(sensor_msgs::msg::JointState::SharedPtr);
    int set_jacobian();
    int get_model_joints();
    void initialize_solver();
    Eigen::VectorXd solve_for_velocity(state, state, int);
    Eigen::VectorXd velocity_to_pos(Eigen::VectorXd&, double);
    pinocchio::Data::Matrix6x get_model_jacobian();
    void set_current_state();
    const state get_state();
    const pinocchio::Model get_model();

private:
    pinocchio::Model m_model;
    pinocchio::Data m_model_data;

    Eigen::VectorXd m_joint_pos;
    Eigen::VectorXd m_joint_vel;
    Eigen::VectorXd m_joint_acc;

    pinocchio::Data::Matrix6x J_left_foot;
    pinocchio::Data::Matrix6x J_right_foot;
    pinocchio::Data::Matrix6x J_com;

    state m_current_state;

    // Solver related variables
    std::shared_ptr<labrob::qpsolvers::QPSolverEigenWrapper<double>> m_qp_solver_ptr;
};

#endif // IK_SOLVER_JACOBIAN_GENERATOR_HPP