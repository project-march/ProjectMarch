#ifndef IK_SOLVER__TASK_HPP
#define IK_SOLVER__TASK_HPP

#define EIGEN_RUNTIME_NO_MALLOC

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "march_shared_msgs/srv/get_node_jacobian.hpp"
#include "march_shared_msgs/srv/get_node_position.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp/rclcpp.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

class Task {
public:
    typedef std::unique_ptr<Task> UniquePtr;
    typedef std::shared_ptr<Task> SharedPtr;

    Task(const std::string& task_name, const std::string& reference_frame, const unsigned int& workspace_dim,
        const unsigned int& configuration_dim, const float& dt);
    ~Task() = default;

    Eigen::VectorXd solveTask();
    Eigen::VectorXd calculateError();
    Eigen::VectorXd calculateDerivativeError(const Eigen::VectorXd& error);
    Eigen::VectorXd calculateIntegralError(const Eigen::VectorXd& error);
    void requestCurrentTask();
    void calculateJacobianInverse();

    std::vector<std::string> getNodeNames() const;
    std::string getTaskName() const;
    unsigned int getTaskM() const;
    unsigned int getTaskN() const;
    double getErrorNorm() const;
    std::vector<double> getError() const;
    Eigen::VectorXd getDesiredTask() const;
    Eigen::MatrixXd getNullspaceProjection() const;
    Eigen::MatrixXd getJacobian() const;
    Eigen::MatrixXd getJacobianInverse() const;
    const std::vector<std::string>* getJointNamesPtr() const;
    const Eigen::VectorXd* getCurrentJointPositionsPtr() const;

    void setTaskName(const std::string& task_name);
    void setNodeNames(const std::vector<std::string>& node_names);
    void setTaskM(const unsigned int& task_m);
    void setTaskN(const unsigned int& task_n);
    void setDt(const float& dt);
    void setGainP(const std::vector<double>& gain_p);
    void setGainD(const std::vector<double>& gain_d);
    void setGainI(const std::vector<double>& gain_i);
    void setDampingCoefficient(const float& damping_coefficient);
    void setDesiredTask(const Eigen::VectorXd& desired_task);
    void setJointNamesPtr(std::vector<std::string>* joint_names);
    void setCurrentJointPositionsPtr(Eigen::VectorXd* current_joint_positions_ptr);

    void setCurrentTask(const Eigen::VectorXd& current_task);
    void setJacobian(const Eigen::MatrixXd& jacobian);
    void setUnitTest(const bool& unit_test);

private:
    Eigen::Vector3d calculateEulerAnglesFromQuaternion(const geometry_msgs::msg::Quaternion& quaternion);
    void sendRequestNodePosition();
    void sendRequestNodeJacobian();

    const uint64_t m_service_timeout = 10; // ms

    rclcpp::Node::SharedPtr m_node;
    rclcpp::Client<march_shared_msgs::srv::GetNodePosition>::SharedPtr m_client_node_position;
    rclcpp::Client<march_shared_msgs::srv::GetNodeJacobian>::SharedPtr m_client_node_jacobian;

    std::string m_task_name;
    std::string m_reference_frame;
    std::vector<std::string> m_node_names;
    unsigned int m_task_m;
    unsigned int m_task_n;
    bool m_unit_test = false;

    std::vector<std::string>* m_joint_names_ptr;
    Eigen::VectorXd* m_current_joint_positions_ptr;
    Eigen::VectorXd m_desired_task;
    Eigen::MatrixXd m_gain_p;
    Eigen::MatrixXd m_gain_d;
    Eigen::MatrixXd m_gain_i;
    float m_dt;
    float m_damping_coefficient;
    Eigen::VectorXd m_integral_error;
    Eigen::VectorXd m_previous_error;
    double m_error_norm;
    std::vector<unsigned int> m_nonzero_gain_p_indices;

    Eigen::VectorXd m_current_task;
    Eigen::MatrixXd m_jacobian;
    Eigen::MatrixXd m_jacobian_inverse;
    Eigen::MatrixXd m_damping_identity;
};

#endif // IK_SOLVER__TASK_HPP