/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef MARCH_STATE_ESTIMATOR__ROBOT_NODE_HPP_
#define MARCH_STATE_ESTIMATOR__ROBOT_NODE_HPP_

#include <string>
#include <vector>
#include <unordered_map>

#include "ginac/ginac.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#define WORKSPACE_DIM 3
#define NO_INERTIA_PARAMS 6



class RobotNode : public std::enable_shared_from_this<RobotNode>
{
public:
    RobotNode() = default;
    ~RobotNode() = default;

    void setName(const std::string & name);
    void setId(const uint64_t & id);
    void setParent(std::shared_ptr<RobotNode> parent);
    void addChild(std::shared_ptr<RobotNode> child);
    void setJointNodes(std::vector<std::shared_ptr<RobotNode>> joint_nodes);
    void setInertia(const std::vector<double> inertia);
    void setOriginPosition(const Eigen::Vector3d & position);
    void setOriginRotation(const Eigen::Matrix3d & rotation);

    // void setExpressionGlobalPosition(const GiNaC::matrix & global_position);
    // void setExpressionGlobalRotation(const GiNaC::matrix & global_rotation);
    // void setExpressionGlobalPositionJacobian(const GiNaC::matrix & global_position_jacobian);
    // void setExpressionGlobalRotationJacobian(const GiNaC::matrix & global_rotation_jacobian);

    void setExpressionGlobalPosition(const std::vector<std::string> & expressions);
    void setExpressionGlobalRotation(const std::vector<std::string> & expressions);
    void setExpressionGlobalPositionJacobian(const std::vector<std::string> & expressions);
    void setExpressionGlobalRotationJacobian(const std::vector<std::string> & expressions);

    std::string getName() const;
    uint64_t getId() const;
    uint64_t getId(const std::string & name) const;
    char getType() const;
    double getMass() const;
    double getLength() const;
    std::vector<double> getJointAxis() const;
    std::vector<double> getInertia() const;
    GiNaC::matrix getOriginPosition() const;
    GiNaC::matrix getOriginRotation() const;
    GiNaC::symbol getJointAngle() const;
    GiNaC::matrix getGlobalPositionExpression() const;
    GiNaC::matrix getGlobalRotationExpression() const;
    Eigen::Vector3d getGlobalPosition(std::unordered_map<std::string, double> joint_positions) const;
    Eigen::Matrix3d getGlobalRotation(std::unordered_map<std::string, double> joint_positions) const;
    Eigen::MatrixXd getGlobalPositionJacobian(std::unordered_map<std::string, double> joint_positions) const;
    Eigen::MatrixXd getGlobalRotationJacobian(std::unordered_map<std::string, double> joint_positions) const;
    std::shared_ptr<RobotNode> getParent() const;
    std::vector<std::weak_ptr<RobotNode>> getChildren() const;
    std::vector<std::string> getJointNames() const;

    void expressRotation();
    void expressKinematics();
    void expressDynamics();

protected:
    std::vector<GiNaC::symbol> getJointAngles() const;
    std::vector<std::shared_ptr<RobotNode>> getJointNodes(std::shared_ptr<RobotNode> parent) const;
    GiNaC::matrix expressGlobalPosition() const;
    GiNaC::matrix expressGlobalRotation() const;
    GiNaC::matrix expressGlobalPositionJacobian() const;
    GiNaC::matrix expressGlobalRotationJacobian() const;

    void setExpression(const std::vector<std::string> & expressions, std::vector<GiNaC::ex> & target);
    Eigen::MatrixXd evaluateExpression(const std::vector<GiNaC::ex> & expressions, 
        const std::unordered_map<std::string, double> & joint_positions,
        const unsigned int & rows, const unsigned int & cols) const;
    GiNaC::lst substituteSymbolsWithJointValues(const std::unordered_map<std::string, double> & joint_positions) const;

    GiNaC::matrix utilConvertEigenToGiNaC(const Eigen::MatrixXd & matrix) const;
    Eigen::Matrix3d utilConvertGiNaCToEigen(const GiNaC::matrix & matrix) const;
    int utilGetJointAxisIndex() const;

    std::string m_name;
    uint64_t m_id;
    char m_type;
    double m_mass;
    double m_length;
    double m_inertia[NO_INERTIA_PARAMS];
    std::shared_ptr<RobotNode> m_parent = nullptr;
    std::vector<std::weak_ptr<RobotNode>> m_children;
    std::vector<std::shared_ptr<RobotNode>> m_joint_nodes;
    std::vector<double> m_joint_axis;

    GiNaC::symbol m_joint_angle;
    GiNaC::matrix m_global_position_vector;
    GiNaC::matrix m_global_rotation_matrix;
    GiNaC::matrix m_origin_position_vector;
    GiNaC::matrix m_origin_rotation_matrix;
    GiNaC::matrix m_global_position_jacobian_matrix;
    GiNaC::matrix m_global_rotation_jacobian_matrix;
    std::vector<GiNaC::symbol> m_joint_angles;
    GiNaC::symtab m_joint_symbols_table;
    GiNaC::lst m_joint_symbols_list;

    std::vector<GiNaC::ex> m_global_position_expressions;
    std::vector<GiNaC::ex> m_global_rotation_expressions;
    std::vector<GiNaC::ex> m_global_position_jacobian_expressions;
    std::vector<GiNaC::ex> m_global_rotation_jacobian_expressions;
};

#endif  // MARCH_STATE_ESTIMATOR__ROBOT_NODE_HPP_