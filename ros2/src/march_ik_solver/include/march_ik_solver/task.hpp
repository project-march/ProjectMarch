#ifndef IK_SOLVER__TASK_HPP
#define IK_SOLVER__TASK_HPP

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "march_shared_msgs/srv/get_task_report.hpp"
#include "rclcpp/rclcpp.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

class Task {
public:
    Task(); // Default constructor.
    Task( // Constructor. TODO: Load the task parameters from a YAML file
        uint8_t task_id, // ID of the task
        std::string task_name, // Name of the task
        uint8_t task_m, // Dimension of the task
        uint8_t task_n // Dimension of the joint space
    );

    std::string getTaskName(); // Get the name of the task
    int getTaskID(); // Get the ID of the task
    int getTaskM(); // Get the dimension of the task
    int getTaskN(); // Get the dimension of the joint space

    Eigen::VectorXd solve(); // Solve the task

    void setDesiredPose(Eigen::VectorXd* desired_pose); // Set the desired pose of the task

    void setGainP(float gain_p); // Set the proportional gain
    // void setGainI(float gain_i);                    // Set the integral gain
    // void setGainD(float gain_d);                    // Set the derivative gain
    void setDampingCoefficient(float damping_coefficient); // Set the damping coefficient

    const Eigen::MatrixXd* getJacobianPtr(); // Get the Jacobian
    const Eigen::MatrixXd* getJacobianInversePtr(); // Get the inverse of Jacobian

private:
    Eigen::VectorXd calculateError(); // Calculate the error
    // Eigen::Vector3f calculateIntegralError();       // Calculate the integral error
    // Eigen::Vector3f calculateDerivativeError();     // Calculate the derivative error
    void calculateJacobianInverse(); // Calculate the inverse of Jacobian
    void sendRequest(); // Send a request to the task server in the state estimation node

    // Create a ROS2 client to communicate with the task server in the state estimation node.
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<march_shared_msgs::srv::GetTaskReport>::SharedPtr client_;

    // Declare variables that define the task.
    std::string task_name_; // Name of the task
    uint8_t task_id_; // ID of the task
    uint8_t task_m_; // Dimension of the task
    uint8_t task_n_; // Dimension of the joint space

    Eigen::VectorXd* desired_pose_ptr_; // Pointer to desired pose of the task
    Eigen::VectorXd current_pose_; // Current pose of the task
    float gain_p_ = 0.0; // Proportional gain. Default value is 0.0
    float gain_i_ = 0.0; // Integral gain. Default value is 0.0
    float gain_d_ = 0.0; // Derivative gain. Default value is 0.0
    float damping_coefficient_ = 0.0; // Damping coefficient. Default value is 0.0
    Eigen::MatrixXd jacobian_; // Jacobian matrix. TODO: Load this from State Estimation Server.
    Eigen::MatrixXd jacobian_inverse_; // Inverse of Jacobian matrix.
    // Eigen::VectorXd previous_error_;        // Previous error
    // Eigen::VectorXd integral_error_;        // Integral error
};

#endif // IK_SOLVER__TASK_HPP