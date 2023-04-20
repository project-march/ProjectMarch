#ifndef TORQUE_CONVERTER_NODE_H
#define TORQUE_CONVERTER_NODE_H

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "march_shared_msgs/msg/ik_solver_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Core>
#include <vector>
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"

using namespace Eigen;

class TorqueConverter : public rclcpp::Node {

public:
    TorqueConverter();
    void load_urdf_model(std::string);
    void set_inertia_matrix();
    // void set_desired_pos(VectorXd desired_pos); // for testing
    VectorXd get_desired_vel();
    VectorXd get_desired_acc();
    

private:
    void trajectory_subscriber_callback(sensor_msgs::msg::JointState::SharedPtr);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_trajectory_subscriber;

    // pinocchio
    pinocchio::Model m_model;
    pinocchio::Data m_model_data;

    MatrixXf m_inertia_matrix;
    MatrixXf m_coriolis_matrix;
    MatrixXf m_gravity_matrix;

    VectorXd m_pos_des; // should be a trajectory of at least 3 angles for each joint
    VectorXd m_vel_des; 
    VectorXd m_acc_des;

    VectorXd find_vel_des();
    VectorXd find_acc_des();

    

    double m_dt = 1; // the timestep between position points


    float convert_to_torque();

    std::vector<VectorXd> torque_des;
    
};

#endif