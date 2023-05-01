#ifndef TORQUE_CONVERTER_H
#define TORQUE_CONVERTER_H

// #include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Core>
#include <vector>
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"

using namespace Eigen;

class TorqueConverter {

public:
    TorqueConverter();
    void load_urdf_model(std::string);
    
    void set_inertia_matrix();
    void set_coriolis_matrix();
    void set_gravity_matrix();

    void set_joint_config(sensor_msgs::msg::JointState::SharedPtr);
    void set_desired_pos(std::vector<double>); 
    void set_desired_vel(std::vector<double>);
    void set_desired_acc(std::vector<double>);

    VectorXd get_desired_vel();
    VectorXd get_desired_acc();

private:

    // pinocchio
    pinocchio::Model m_model;
    pinocchio::Data m_model_data;
    Eigen::VectorXd m_q; // current joint positions (need to be loaded in)
    Eigen::VectorXd m_v; // current joint velocities

    VectorXd m_pos_des; // should be a trajectory of at least 3 angles for each joint
    VectorXd m_vel_des; // desired velocity calculated with forward euler from pos_des
    VectorXd m_acc_des; // desired acceleration calculated with forward euler from vel_des

    VectorXd m_torque_des; // torque trajectory
    VectorXd convert_to_torque(); // the main function

    
    
};

#endif