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

}

void TorqueConverter::load_urdf_model(std::string urdf_filename)
{
     // create model from urdf
    pinocchio::urdf::buildModel(urdf_filename, m_model);
    
    m_model_data = pinocchio::Data(m_model);

    m_q = Eigen::VectorXd::Zero(m_model.nq); //to ensure it is the same size as the amount of joints
    m_v = Eigen::VectorXd::Zero(m_model.nq);

    m_pos_des_history.resize(2);
    m_pos_des_history.setZero();
}

void TorqueConverter::set_inertia_matrix()
{
     // The inertia matrix is saved in m_model_data.M

    // RCLCPP_INFO(rclcpp::get_logger("m_q_size_logger"), "mq size inertia in generator is %i", m_q.size());

    pinocchio::crba(m_model, m_model_data, m_q);
}

void TorqueConverter::set_coriolis_matrix()
{
    // The coriolis matrix is saved in m_model_data.C
    // RCLCPP_INFO(rclcpp::get_logger("m_q_size_logger"), "mq size coriolis in set coriolis is %i", m_q.size());
    // RCLCPP_INFO(get_logger(), "m_q size: %i", m_q.size());

    pinocchio::computeCoriolisMatrix(m_model, m_model_data, m_q, m_v); 
}

void TorqueConverter::set_gravity_matrix()
{
    //The gravity matrix is saved in m_model_data.g
    pinocchio::computeGeneralizedGravity(m_model, m_model_data, m_q); 
}

void TorqueConverter::set_joint_config(sensor_msgs::msg::JointState::SharedPtr msg)
{
    // RCLCPP_INFO(rclcpp::get_logger("m_q_size_logger"), "m_model.njoints = %i", m_model.njoints);
    for (int i = 1; i < m_model.njoints; i++) {
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
            m_q[i-1] = msg->position[joint_index];
            m_v[i-1] = msg->velocity[joint_index];
        } else {
            // Error message when joint is not found
            throw std::runtime_error("Joint " + m_model.names[i] + " not found in JointState message");
        }

    }
    // RCLCPP_INFO(rclcpp::get_logger("m_q_size_logger"), "mq size after joint config is %i", m_q.size());
}

void TorqueConverter::set_desired_pos(std::vector<double> desired_pos) 
{
    m_pos_des = VectorXd::Map(desired_pos.data(), desired_pos.size()); //maps std::vector to Eigen VectorXd
}

// void TorqueConverter::set_desired_vel(std::vector<double> desired_vel)
// {
//     // m_vel_des = VectorXd::Map(desired_vel.data(), desired_vel.size()); //maps std::vector to Eigen VectorXd
// }

void TorqueConverter::set_desired_acc(std::vector<double> desired_acc)
{
    // RCLCPP_INFO(rclcpp::get_logger("m_q_size_logger"), "desired_acc size = %i", desired_acc.size());
    m_acc_des = VectorXd::Map(desired_acc.data(), desired_acc.size()); //maps std::vector to Eigen VectorXd
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
    m_torque_des.resize(1);
    // RCLCPP_INFO(rclcpp::get_logger("m_q_size_logger"), "m_model_data.M size = %i", m_model_data.M.cols());
    m_torque_des << 3; //m_model_data.M * m_acc_des + m_model_data.C * m_vel_des + m_model_data.g; // torque = M(q)*ddq + C(q,dq) *dq + G(q)

return m_torque_des;}

VectorXd TorqueConverter::set_vel_des(std::vector<double> desired_pos)
{
    m_vel_des.resize(1);
    VectorXd pos_des = VectorXd::Map(desired_pos.data(), desired_pos.size());

    RCLCPP_INFO(rclcpp::get_logger("set_Vel_des"), "1 pos_des_history = %i", desired_pos.size());
    RCLCPP_INFO(rclcpp::get_logger("set_Vel_des"), "pos_des_history = %d", pos_des[0]); // this isn't printing!
    

    m_pos_des_history[1] = m_pos_des_history[0];
    m_pos_des_history[0] = pos_des[0];

    m_vel_des << (m_pos_des_history[0] - m_pos_des_history[1])/m_dt;
    RCLCPP_INFO(rclcpp::get_logger("Vel_des"), "m_vel_des = %d", m_vel_des[0]);
    // for (int i = 1; i < m_pos_des.size(); i++)
    // {
    //     m_vel_des.coeffRef(i-1) = (m_pos_des.coeffRef(i) - m_pos_des.coeffRef(i-1)) / m_dt;
    // }
    return m_vel_des;
}

VectorXd TorqueConverter::find_acc_des(std::vector<double> desired_vel)
{
    m_acc_des.resize(1);

    m_vel_des_history[1] = m_vel_des_history[0];
    m_vel_des_history[0] = desired_vel[0];

    m_acc_des << (m_vel_des_history[0] - m_vel_des_history[1])/m_dt;

    // m_acc_des.coeffRef(0) = (m_vel_des.coeffRef(0) - 0 / m_dt);

//     for (int i = 2; i < m_vel_des.size(); i++)
//     {
//         m_acc_des.coeffRef(i-1) = (m_vel_des.coeffRef(i-1) - m_vel_des.coeffRef(i-2)) / m_dt;
//     }
    return m_acc_des;
 }