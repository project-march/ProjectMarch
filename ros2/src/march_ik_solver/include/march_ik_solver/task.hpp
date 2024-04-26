#ifndef IK_SOLVER__TASK_HPP
#define IK_SOLVER__TASK_HPP

#define EIGEN_RUNTIME_NO_MALLOC

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "pinocchio/algorithm/kinematics.hpp"

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

    std::vector<std::string> getNodeNames() const;
    inline std::vector<int> getJointIndices() const { return m_joint_indices; }
    std::string getTaskName() const;
    unsigned int getTaskM() const;
    unsigned int getTaskN() const;
    double getErrorNorm() const;
    Eigen::VectorXd getDesiredTask() const;
    Eigen::MatrixXd getNullspaceProjection() const;
    Eigen::MatrixXd getJacobian() const;
    Eigen::MatrixXd getJacobianInverse() const;
    const std::vector<std::string>* getJointNamesPtr() const;
    const Eigen::VectorXd* getCurrentJointPositionsPtr() const;

    void setTaskName(const std::string& task_name);
    void setNodeNames(const std::vector<std::string>& node_names);
    inline void setJointIndices(const std::vector<int>& joint_indices) { m_joint_indices = joint_indices; }
    void setTaskM(const unsigned int& task_m);
    void setTaskN(const unsigned int& task_n);
    void setDt(const float& dt);
    void setGainP(const std::vector<double>& gain_p);
    void setGainD(const std::vector<double>& gain_d);
    void setGainI(const std::vector<double>& gain_i);
    void setDampingCoefficient(const float& damping_coefficient);
    inline void setDesiredTask(const Eigen::VectorXd& desired_task) { m_desired_task = desired_task; }
    inline void setJointNamesPtr(std::vector<std::string>* joint_names) { m_joint_names_ptr = joint_names; }
    inline void setCurrentJointPositionsPtr(Eigen::VectorXd* current_joint_positions_ptr) { m_current_joint_positions_ptr = current_joint_positions_ptr; }

    inline void setCurrentTask(const Eigen::VectorXd& current_task) { m_current_task = current_task; }
    inline void setJacobian(const Eigen::MatrixXd& jacobian) { m_jacobian = jacobian; }
    inline void setUnitTest(const bool& unit_test) { m_unit_test = unit_test; }

private:
    void computeCurrentTask();
    void computeJacobian();
    void computeJacobianInverse();

    std::string m_task_name;
    std::string m_reference_frame;
    unsigned int m_task_m;
    unsigned int m_task_n;
    std::vector<std::string> m_node_names;
    std::vector<int> m_joint_indices;
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

    pinocchio::Model m_model;
    std::unique_ptr<pinocchio::Data> m_data;
    
    const unsigned int EUCLIDEAN_SIZE = 3;
    const unsigned int SE3_SIZE = 6;
};

#endif // IK_SOLVER__TASK_HPP