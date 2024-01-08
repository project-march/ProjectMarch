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
    Task() = default;
    Task(uint8_t task_id, std::string task_name, uint8_t task_m, uint8_t task_n, std::vector<std::string> node_names);
    ~Task() = default;

    std::string getTaskName();
    unsigned int getTaskID();
    int getTaskM();
    int getTaskN();
    double getErrorNorm();

    Eigen::VectorXd solve();

    void setCurrentJointNamesPtr(std::vector<std::string> * current_joint_names);
    void setCurrentJointPositionsPtr(Eigen::VectorXd* current_joint_positions);
    void setDesiredPosesPtr(std::vector<Eigen::VectorXd> * desired_poses_ptr);

    void setGainP(float gain_p);
    void setDampingCoefficient(float damping_coefficient);

    const Eigen::MatrixXd* getJacobianPtr();
    const Eigen::MatrixXd* getJacobianInversePtr();

private:
    Eigen::VectorXd calculateError();
    void calculateCurrentPose();
    void calculateJacobian();
    void calculateJacobianInverse();
    void sendRequest();
    void sendRequestNodePosition();
    void sendRequestNodeJacobian();

    rclcpp::Node::SharedPtr m_node;
    rclcpp::Client<march_shared_msgs::srv::GetTaskReport>::SharedPtr m_client;
    rclcpp::Client<march_shared_msgs::srv::GetNodePosition>::SharedPtr m_client_node_position;
    rclcpp::Client<march_shared_msgs::srv::GetNodeJacobian>::SharedPtr m_client_node_jacobian;

    std::string m_task_name;
    uint8_t m_task_id;
    uint8_t m_task_m;
    uint8_t m_task_n;
    std::vector<std::string> m_node_names;

    std::vector<std::string> * m_current_joint_names_ptr;
    Eigen::VectorXd * m_current_joint_positions_ptr;
    std::vector<Eigen::VectorXd> * m_desired_poses_ptr;
    Eigen::VectorXd m_current_pose;
    double m_error_norm;
    float m_gain_p = 0.0;
    float m_damping_coefficient = 0.0;
    Eigen::MatrixXd m_jacobian;
    Eigen::MatrixXd m_jacobian_inverse;
};

#endif // IK_SOLVER__TASK_HPP