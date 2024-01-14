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
    // Task() = default;
    Task(unsigned int task_id, std::string task_name, unsigned int task_m, unsigned int task_n, std::vector<std::string> node_names);
    ~Task() = default;

    Eigen::VectorXd solve();
    Eigen::VectorXd calculateError();
    Eigen::VectorXd calculateDerivativeError(const Eigen::VectorXd & error);
    Eigen::VectorXd calculateIntegralError(const Eigen::VectorXd & error);
    void calculateJacobianInverse();

    std::string getTaskName() const;
    unsigned int getTaskID() const;
    unsigned int getTaskM() const;
    unsigned int getTaskN() const;
    std::vector<std::string> getNodeNames() const;

    void setTaskName(const std::string & task_name);
    void setTaskID(const unsigned int & task_id);
    void setTaskM(const unsigned int & task_m);
    void setTaskN(const unsigned int & task_n);
    void setNodeNames(const std::vector<std::string> & node_names);
    
    double getErrorNorm() const;
    const Eigen::VectorXd * getCurrentJointPositionsPtr() const;
    const std::vector<std::string> * getCurrentJointNamesPtr() const;
    const std::vector<Eigen::VectorXd> * getDesiredPosesPtr() const;
    const Eigen::MatrixXd * getJacobianPtr();
    const Eigen::MatrixXd * getJacobianInversePtr();

    void setCurrentJointPositionsPtr(Eigen::VectorXd* current_joint_positions);
    void setCurrentJointNamesPtr(std::vector<std::string> * current_joint_names);
    void setDesiredPosesPtr(std::vector<Eigen::VectorXd> * desired_poses_ptr);
    void setCurrentPose(const Eigen::VectorXd & current_pose);
    void setJacobian(const Eigen::MatrixXd & jacobian);
    void setGainP(const float & gain_p);
    void setGainD(const float & gain_d);
    void setGainI(const float & gain_i);
    void setDt(const float & dt);
    void setDampingCoefficient(const float & damping_coefficient);
    void setUnitTest(const bool & unit_test);

private:
    void sendRequestNodePosition();
    void sendRequestNodeJacobian();

    rclcpp::Node::SharedPtr m_node;
    rclcpp::Client<march_shared_msgs::srv::GetNodePosition>::SharedPtr m_client_node_position;
    rclcpp::Client<march_shared_msgs::srv::GetNodeJacobian>::SharedPtr m_client_node_jacobian;

    std::string m_task_name;
    unsigned int m_task_id;
    unsigned int m_task_m;
    unsigned int m_task_n;
    std::vector<std::string> m_node_names;
    bool m_unit_test = false;

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