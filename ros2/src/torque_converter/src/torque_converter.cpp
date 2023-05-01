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
    m_q = Eigen::VectorXd::Zero(m_model.nq); //to ensure it is the same size as the amount of joints
    m_v = Eigen::VectorXd::Zero(m_model.nq);
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
    pinocchio::computeCoriolisMatrix(m_model, m_model_data, m_q, m_v); 
}

void TorqueConverter::set_gravity_matrix()
{
    //The gravity matrix is saved in m_model_data.g
    pinocchio::computeGeneralizedGravity(m_model, m_model_data, m_q); 
}

void TorqueConverter::set_joint_config(sensor_msgs::msg::JointState::SharedPtr msg)
{
    for (int i = 0; i < m_model.njoints; i++) {
        // Find the index of the joint in the JointState message
        int joint_index = -1;
        for (long unsigned int j = 0; j < msg->name.size(); j++) {
            if (m_model.names[i] == msg->name[j]) {
                joint_index = j;
                break;
            }
        }
        // If the joint was found, set its position in the m_q vector
        if (joint_index >= 0) {
            m_q[i] = msg->position[joint_index];
            m_v[i] = msg->velocity[joint_index];
        } else {
            // Error message when joint is not found
            throw std::runtime_error("Joint " + m_model.names[i] + " not found in JointState message");
        }
    }
}

void TorqueConverter::set_desired_pos(std::vector<double> desired_pos) 
{
    m_pos_des = VectorXd::Map(desired_pos.data(), desired_pos.size()); //maps std::vector to Eigen VectorXd
}

void TorqueConverter::set_desired_vel(std::vector<double> desired_vel)
{
    m_vel_des = VectorXd::Map(desired_vel.data(), desired_vel.size()); //maps std::vector to Eigen VectorXd
}

void TorqueConverter::set_desired_acc(std::vector<double> desired_acc)
{
    m_vel_des = VectorXd::Map(desired_acc.data(), desired_acc.size()); //maps std::vector to Eigen VectorXd
}

VectorXd TorqueConverter::get_desired_vel()
{
    return m_vel_des;
}

VectorXd TorqueConverter::get_desired_acc()
{
    return m_acc_des;
}

VectorXd TorqueConverter::convert_to_torque()
{
    m_torque_des = m_model_data.M * m_acc_des + m_model_data.C * m_vel_des + m_model_data.g; // torque = M(q)*ddq + C(q,dq) *dq + G(q)

return m_torque_des;}

// VectorXd TorqueConverter::find_vel_des()
// {
//     m_vel_des.resize(m_pos_des.size()-1);

//     for (int i = 1; i < m_pos_des.size(); i++)
//     {
//         m_vel_des.coeffRef(i-1) = (m_pos_des.coeffRef(i) - m_pos_des.coeffRef(i-1)) / m_dt;
//     }
//     return m_vel_des;
// }

// VectorXd TorqueConverter::find_acc_des()
// {
//     m_acc_des.resize(m_vel_des.size()-1);

//     m_acc_des.coeffRef(0) = (m_vel_des.coeffRef(0) - 0 / m_dt);

//     for (int i = 2; i < m_vel_des.size(); i++)
//     {
//         m_acc_des.coeffRef(i-1) = (m_vel_des.coeffRef(i-1) - m_vel_des.coeffRef(i-2)) / m_dt;
//     }
//     return m_acc_des;
// }