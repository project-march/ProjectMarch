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
    : Node("torque_converter")
{
    // create subscriber
    m_trajectory_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
        "/torque_trajectory", 10, std::bind(&TorqueConverter::trajectory_subscriber_callback, this, _1));

}

void TorqueConverter::load_urdf_model(std::string urdf_filename)
{
     // create model from urdf
    pinocchio::urdf::buildModel(urdf_filename, m_model);
    m_model_data = pinocchio::Data(m_model);

    // initialize inertia matrix
    pinocchio::crba(m_model, m_model_data, Eigen::VectorXd::Zero(m_model.nq)); // inertia matrix is saved in m_model_data.M, but crba only fills in upper triangle
    m_model_data.M.triangularView<Eigen::StrictlyLower>() = m_model_data.M.transpose().triangularView<Eigen::StrictlyLower>(); // to make the inertia matrix square
}

void set_inertia_matrix()
{
    pinocchio::crba(m_model, m_model_data, m_model_data.q); // inertia matrix is saved in m_model_data.M
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


// void TorqueConverter::set_desired_pos(VectorXd desired_pos) // for testing
// {
//     m_pos_des = desired_pos; 
// }


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

    for (int i = 1; i < m_vel_des.size(); i++)
    {
        m_acc_des.coeffRef(i-1) = (m_vel_des.coeffRef(i) - m_vel_des.coeffRef(i-1)) / m_dt;
    }
    return m_acc_des;
}

float TorqueConverter::convert_to_torque()
{

return 0.5;}
