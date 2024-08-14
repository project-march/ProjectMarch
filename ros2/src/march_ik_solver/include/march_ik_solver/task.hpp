/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef IK_SOLVER__TASK_HPP
#define IK_SOLVER__TASK_HPP

#pragma once
#define EIGEN_RUNTIME_NO_MALLOC

#include <string>
#include <vector>

#include "march_shared_msgs/msg/iks_task_status.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

class Task {
public:
    typedef std::unique_ptr<Task> UniquePtr;
    typedef std::shared_ptr<Task> SharedPtr;

    Task();
    ~Task();

    Eigen::VectorXd solveTask();
    Eigen::VectorXd calculateError();
    Eigen::VectorXd calculateDerivativeError(const Eigen::VectorXd& error);
    Eigen::VectorXd calculateIntegralError(const Eigen::VectorXd& error);
    void computeCurrentTask();
    void configureIkSolverVariables();
    inline bool isConverged() { return m_error_norm < m_convergence_threshold; }

    inline std::vector<int> getJointIndices() const { return m_joint_indices; }
    inline std::string getTaskName() const { return m_task_name; }
    inline unsigned int getTaskM() const { return m_task_m; }
    inline unsigned int getTaskN() const { return m_task_n; }
    inline double getWeight() const { return m_weight; }
    double getErrorNorm() const;
    std::vector<double> getError() const;
    march_shared_msgs::msg::IksTaskStatus getTaskStatus() const;
    Eigen::MatrixXd getNullspaceProjection() const;
    inline Eigen::VectorXd getDesiredTask() const { return m_desired_task; }
    inline Eigen::MatrixXd getJacobian() const { return m_jacobian; }
    inline Eigen::MatrixXd getJacobianInverse() const { return m_jacobian_inverse; }
    inline const std::vector<std::string>* getJointNamesPtr() const { return m_joint_names_ptr; }
    inline const Eigen::VectorXd* getCurrentJointPositionsPtr() const { return m_current_joint_positions_ptr; }

    void setTaskName(const std::string& task_name);
    inline void setJointIndices(const std::vector<int>& joint_indices) { m_joint_indices = joint_indices; }
    void setTaskM(const unsigned int& task_m);
    void setTaskN(const unsigned int& task_n);
    void setGainP(const std::vector<double>& gain_p);
    void setGainD(const std::vector<double>& gain_d);
    void setGainI(const std::vector<double>& gain_i);
    void setDampingCoefficient(const double& damping_coefficient);
    inline void setDt(const double& dt) { m_dt = dt; }
    inline void setConvergenceThreshold(const double& convergence_threshold) { m_convergence_threshold = convergence_threshold; }
    inline void setWeight(const double& weight) { m_weight = weight; }
    inline void setDesiredTask(const Eigen::VectorXd& desired_task) { m_desired_task = desired_task; }
    inline void setJointNamesPtr(std::vector<std::string>* joint_names) { m_joint_names_ptr = joint_names; }
    inline void setCurrentJointPositionsPtr(Eigen::VectorXd* current_joint_positions_ptr) { m_current_joint_positions_ptr = current_joint_positions_ptr; }
    inline void setCurrentWorldToBaseOrientation(const Eigen::Matrix3d& current_world_to_base_orientation) { m_current_world_to_base_orientation = current_world_to_base_orientation; }

    inline void setCurrentTask(const Eigen::VectorXd& current_task) { m_current_task = current_task; }
    inline void setJacobian(const Eigen::MatrixXd& jacobian) { m_jacobian = jacobian; }
    inline void setUnitTest(const bool& unit_test) { m_unit_test = unit_test; }

protected:
    virtual void computeCurrentTaskCoordinates();
    virtual void computeCurrentTaskJacobian();
    void computeJacobianInverse();

    std::string m_task_name;
    std::string m_reference_frame;
    unsigned int m_task_m;
    unsigned int m_task_n;
    std::vector<int> m_joint_indices;
    bool m_unit_test = false;

    std::vector<std::string>* m_joint_names_ptr;
    Eigen::VectorXd* m_current_joint_positions_ptr;
    Eigen::Matrix3d m_current_world_to_base_orientation;
    Eigen::VectorXd m_desired_task;
    Eigen::MatrixXd m_gain_p;
    Eigen::MatrixXd m_gain_d;
    Eigen::MatrixXd m_gain_i;
    double m_dt;
    double m_damping_coefficient;
    double m_convergence_threshold;
    double m_weight;
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
    const unsigned int ROTATION_ROLL_INDEX = 0;
    const unsigned int ROTATION_PITCH_INDEX = 1;
    const unsigned int ROTATION_YAW_INDEX = 2;
};

#endif // IK_SOLVER__TASK_HPP