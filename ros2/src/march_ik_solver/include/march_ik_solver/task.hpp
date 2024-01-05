#ifndef IK_SOLVER__TASK_HPP
#define IK_SOLVER__TASK_HPP

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "march_shared_msgs/srv/get_task_report.hpp"
#include "march_shared_msgs/srv/get_node_position.hpp"
#include "march_shared_msgs/srv/get_node_jacobian.hpp"
#include "rclcpp/rclcpp.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

class Task {
public:
    Task() = default; // Default constructor.
    Task( // Constructor. TODO: Load the task parameters from a YAML file
        uint8_t task_id, // ID of the task
        std::string task_name, // Name of the task
        uint8_t task_m, // Dimension of the task
        uint8_t task_n, // Dimension of the joint space
        std::vector<std::string> node_names // Names of the nodes
    );
    ~Task() = default; // Default destructor.

    std::string getTaskName(); // Get the name of the task
    unsigned int getTaskID(); // Get the ID of the task
    int getTaskM(); // Get the dimension of the task
    int getTaskN(); // Get the dimension of the joint space
    double getErrorNorm(); // Get the norm of the error

    Eigen::VectorXd solve(); // Solve the task

    // Eigen::VectorXd getPose(const Eigen::VectorXd * joint_positions);
    void setCurrentJointNamesPtr(std::vector<std::string> * current_joint_names); // Set the current joint names
    void setCurrentJointPositionsPtr(Eigen::VectorXd* current_joint_positions); // Set the current joint positions
    void setDesiredPosesPtr(std::vector<Eigen::VectorXd> * desired_poses_ptr); // Set the desired pose of the task

    void setGainP(float gain_p); // Set the proportional gain
    // void setGainI(float gain_i);                    // Set the integral gain
    // void setGainD(float gain_d);                    // Set the derivative gain
    void setDampingCoefficient(float damping_coefficient); // Set the damping coefficient

    const Eigen::MatrixXd* getJacobianPtr(); // Get the Jacobian
    const Eigen::MatrixXd* getJacobianInversePtr(); // Get the inverse of Jacobian

private:
    Eigen::VectorXd calculateError(); // Calculate the error
    // Eigen::VectorXd calculateIntegralError();       // Calculate the integral error
    // Eigen::VectorXd calculateDerivativeError();     // Calculate the derivative error
    void calculateCurrentPose(); // Calculate the current pose of the task
    void calculateJacobian(); // Calculate the Jacobian
    void calculateJacobianInverse(); // Calculate the inverse of Jacobian
    void sendRequest(); // Send a request to the task server in the state estimation node
    void sendRequestNodePosition(); // Send a request to the node position server in the state estimation node
    void sendRequestNodeJacobian(); // Send a request to the node Jacobian server in the state estimation node

    // Create a ROS2 client to communicate with the task server in the state estimation node.
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<march_shared_msgs::srv::GetTaskReport>::SharedPtr client_;
    rclcpp::Client<march_shared_msgs::srv::GetNodePosition>::SharedPtr client_node_position_;
    rclcpp::Client<march_shared_msgs::srv::GetNodeJacobian>::SharedPtr client_node_jacobian_;

    // Declare variables that define the task.
    std::string task_name_; // Name of the task
    uint8_t task_id_; // ID of the task
    uint8_t task_m_; // Dimension of the task
    uint8_t task_n_; // Dimension of the joint space
    std::vector<std::string> node_names_; // Names of the nodes

    std::vector<std::string> * current_joint_names_ptr_; // Pointer to current joint names
    Eigen::VectorXd * current_joint_positions_ptr_; // Pointer to current joint positions
    std::vector<Eigen::VectorXd> * desired_poses_ptr_; // Pointer to desired pose of the task
    Eigen::VectorXd current_pose_; // Current pose of the task
    double error_norm_; // Norm of the error
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