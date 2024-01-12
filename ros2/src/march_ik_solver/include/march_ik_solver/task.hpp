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

    Eigen::VectorXd solve();

    std::string getTaskName() const;
    unsigned int getTaskID() const;
    int getTaskM() const;
    int getTaskN() const;
    double getErrorNorm() const;
    const Eigen::MatrixXd * getJacobianPtr();
    const Eigen::MatrixXd * getJacobianInversePtr();

    void setCurrentJointNamesPtr(std::vector<std::string> * current_joint_names);
    void setCurrentJointPositionsPtr(Eigen::VectorXd* current_joint_positions);
    void setDesiredPosesPtr(std::vector<Eigen::VectorXd> * desired_poses_ptr);
    void setCurrentPoses(const Eigen::VectorXd & current_pose);
    void setGainP(const float & gain_p);
    void setGainD(const float & gain_d);
    void setGainI(const float & gain_i);
    void setDt(const float & dt);
    void setDampingCoefficient(const float & damping_coefficient);

private:
    Eigen::VectorXd calculateError();
    Eigen::VectorXd calculateDerivativeError(const Eigen::VectorXd & error);
    Eigen::VectorXd calculateIntegralError(const Eigen::VectorXd & error);
    void calculateJacobianInverse();
    void sendRequestNodePosition();
    void sendRequestNodeJacobian();

    rclcpp::Node::SharedPtr m_node;
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
    float m_dt = 0.0;
    float m_gain_p = 0.0;
    float m_gain_d = 0.0;
    float m_gain_i = 0.0;
    float m_damping_coefficient = 0.0;
    Eigen::VectorXd m_integral_error;
    Eigen::VectorXd m_previous_error;
    Eigen::MatrixXd m_jacobian;
    Eigen::MatrixXd m_jacobian_inverse;
};

#endif // IK_SOLVER__TASK_HPP