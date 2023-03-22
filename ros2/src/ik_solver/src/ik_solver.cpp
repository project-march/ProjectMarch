//
// Created by Tessel & Jack March 6th 2023
//
#include "ik_solver/ik_solver.hpp"

IkSolver::IkSolver()
    {
    
    }

void IkSolver::load_urdf_model(std::string urdf_filename)
    {
    pinocchio::urdf::buildModel(urdf_filename, m_model);
    // pinocchio::buildModels::manipulator(m_model);
    m_model_data = pinocchio::Data(m_model);
    Jfoot.resize(6, m_model.nv);
    Jfoot.setZero();
    }

void IkSolver::set_joint_configuration()
    {
    m_joint_pos = pinocchio::randomConfiguration(m_model);
    m_joint_vel = Eigen::VectorXd::Zero(m_model.nv);
    m_joint_acc = Eigen::VectorXd::Zero(m_model.nv);

    }

int IkSolver::set_jacobian()
    {
    try
        {
        // First, apply forward kinematics
        // pinocchio::forwardKinematics(m_model,m_model_data,m_joint_pos);
        pinocchio::computeJointJacobian(m_model, m_model_data, m_joint_pos, 4, Jfoot);
        
        return 0;
        }
    catch(...)
        {
        // Return Error CODE 1::JOINTS NOT INITIALIZED
        return 1;
        }
    return 1;
    }

int IkSolver::get_model_joints()
    {
    try
    {
        return m_model.nv;
    }
    catch(...)
    {
        return -1;
    }
    
    }

pinocchio::Data::Matrix6x IkSolver::get_model_jacobian()
    {
    return Jfoot;
    }