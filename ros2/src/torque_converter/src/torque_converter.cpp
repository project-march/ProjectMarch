#include "torque_converter/torque_converter.hpp"
#include <cstdio>
#include <chrono>
#include <cstdio>
#include <string>
#include <vector>
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

TorqueConverter::TorqueConverter()
{
    // need a subscriber for m_q , the current joint positions    
    m_q = Eigen::VectorXd::Zero(m_model.nq); //to ensure it is the same size as the amount of joints

}

void TorqueConverter::load_urdf_model(std::string urdf_filename)
{
     // create model from urdf
    pinocchio::urdf::buildModel(urdf_filename, m_model);
    m_model_data = pinocchio::Data(m_model);
}

void TorqueConverter::set_inertia_matrix()
{
     // The inertia matrix is saved in m_model_data.M
    pinocchio::crba(m_model, m_model_data, m_q);
}

void TorqueConverter::set_coriolis_matrix()
{
    // The coriolis matrix is saved in m_model_data.C
    pinocchio::computeCoriolisMatrix(m_model, m_model_data, m_q, m_vel_des); 
}

void TorqueConverter::set_gravity_matrix()
{
    //The gravity matrix is saved in m_model_data.g
    pinocchio::computeGeneralizedGravity(m_model, m_model_data, m_q); 
}

void TorqueConverter::set_desired_pos(VectorXd desired_pos) // for testing
{
    m_pos_des = desired_pos; 
}

VectorXd TorqueConverter::get_desired_vel()
{
    m_vel_des = find_vel_des();
    return m_vel_des;
}

VectorXd TorqueConverter::get_desired_acc()
{
    m_acc_des = find_acc_des();
    return m_acc_des;
}

void TorqueConverter::trajectory_subscriber_callback(sensor_msgs::msg::JointState::SharedPtr msg)
{
}

VectorXd TorqueConverter::find_vel_des()
{
    m_vel_des.resize(m_pos_des.size()-1);

    for (int i = 1; i < m_pos_des.size(); i++)
    {
        m_vel_des.coeffRef(i-1) = (m_pos_des.coeffRef(i) - m_pos_des.coeffRef(i-1)) / m_dt;
    }
    return m_vel_des;
}

VectorXd TorqueConverter::find_acc_des()
{
    m_acc_des.resize(m_vel_des.size()-1);

    m_acc_des.coeffRef(0) = (m_vel_des.coeffRef(0) - 0 / m_dt);

    for (int i = 2; i < m_vel_des.size(); i++)
    {
        m_acc_des.coeffRef(i-1) = (m_vel_des.coeffRef(i-1) - m_vel_des.coeffRef(i-2)) / m_dt;
    }
    return m_acc_des;
}

VectorXd TorqueConverter::convert_to_torque()
{
    m_torque_des = m_model_data.M * m_acc_des + m_model_data.C * m_vel_des + m_model_data.g;

return m_torque_des;}
