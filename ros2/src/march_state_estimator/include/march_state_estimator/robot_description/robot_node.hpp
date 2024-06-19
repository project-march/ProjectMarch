/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef MARCH_STATE_ESTIMATOR__ROBOT_DESCRIPTION__ROBOT_NODE_HPP_
#define MARCH_STATE_ESTIMATOR__ROBOT_DESCRIPTION__ROBOT_NODE_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "ginac/ginac.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#define WORKSPACE_DIM 3
#define NO_INERTIA_PARAMS 6

class RobotNode : public std::enable_shared_from_this<RobotNode> {
public:
    typedef std::shared_ptr<RobotNode> SharedPtr;
    typedef std::weak_ptr<RobotNode> WeakPtr;
    typedef std::unordered_map<std::string, double> QuaternionNameToValueMap;
    typedef std::unordered_map<std::string, double> JointNameToValueMap;
    typedef GiNaC::symbol JointSymbol;
    typedef std::vector<JointSymbol> JointSymbols;
    typedef std::shared_ptr<GiNaC::symbol> JointSymbolPtr; // TODO

    using EvaluateJacobianPtr = Eigen::MatrixXd (RobotNode::*)(const JointNameToValueMap&) const;
    using EvaluateDynamicalTorquePtr
        = Eigen::VectorXd (RobotNode::*)(const JointNameToValueMap&, const JointNameToValueMap&, const JointNameToValueMap&) const;

    RobotNode() = default;
    ~RobotNode() = default;

    void setName(const std::string& name);
    void setId(const uint64_t& id);
    void setParent(SharedPtr parent);
    void addChild(SharedPtr child);
    void setJointNodes(std::vector<SharedPtr> absolute_joint_nodes, std::vector<SharedPtr> relative_joint_nodes);

    void setExpressionRelativeInertia(const std::string& expression);
    void setExpressionGlobalPosition(const std::vector<std::string>& expressions);
    void setExpressionGlobalVelocity(const std::vector<std::string>& expressions);
    void setExpressionGlobalAcceleration(const std::vector<std::string>& expressions);
    void setExpressionGlobalRotation(const std::vector<std::string>& expressions);
    void setExpressionGlobalPositionJacobian(const std::vector<std::string>& expressions);
    void setExpressionGlobalRotationJacobian(const std::vector<std::string>& expressions);
    void setExpressionDynamicalTorque(const std::vector<std::string>& expressions);

    std::string getName() const;
    char getType() const;
    SharedPtr getParent() const;
    JointSymbol getJointPosition() const;
    JointSymbol getJointVelocity() const;
    JointSymbol getJointAcceleration() const;

    virtual Eigen::Vector3d getGlobalPosition(const JointNameToValueMap& joint_positions) const;
    Eigen::Vector3d getGlobalVelocity(const JointNameToValueMap& joint_positions, 
        const JointNameToValueMap& joint_velocities) const;
    Eigen::Vector3d getGlobalAcceleration(const JointNameToValueMap& joint_positions, 
        const JointNameToValueMap& joint_velocities, const JointNameToValueMap& joint_accelerations) const;
    Eigen::Matrix3d getGlobalRotation(const JointNameToValueMap& joint_positions) const;
    virtual Eigen::MatrixXd getGlobalPositionJacobian(const JointNameToValueMap& joint_positions) const;
    Eigen::MatrixXd getGlobalRotationJacobian(const JointNameToValueMap& joint_positions) const;
    Eigen::VectorXd getDynamicalTorque(const JointNameToValueMap& joint_positions, const JointNameToValueMap& joint_velocities,
        const JointNameToValueMap& joint_accelerations) const;
    double getDynamicalJointAcceleration(double joint_torque, const JointNameToValueMap& joint_positions) const;

    std::vector<RobotNode::WeakPtr> getChildren() const;
    std::vector<std::string> getJointNames() const;
    std::vector<std::string> getRelativeJointNames() const;
    JointNameToValueMap getAbsoluteJointValues(const JointNameToValueMap& joint_values) const;
    Eigen::VectorXd convertAbsoluteJointValuesToVectorXd(const JointNameToValueMap& joint_values) const;

protected:
    void setExpression(const std::vector<std::string>& expressions, std::vector<GiNaC::ex>& target);
    Eigen::MatrixXd substituteExpression(const std::vector<GiNaC::ex>& expressions, const unsigned int& rows,
        const unsigned int& cols, const GiNaC::lst& substitutions) const;
    Eigen::MatrixXd evaluateExpression(const std::vector<GiNaC::ex>& expressions,
        const std::vector<RobotNode::SharedPtr> joint_nodes, const unsigned int& rows, const unsigned int& cols,
        const JointNameToValueMap& joint_positions) const;
    Eigen::MatrixXd evaluateExpression(const std::vector<GiNaC::ex>& expressions,
        const std::vector<RobotNode::SharedPtr> joint_nodes, const unsigned int& rows, const unsigned int& cols,
        const JointNameToValueMap& joint_positions, const JointNameToValueMap& joint_velocities) const;
    Eigen::MatrixXd evaluateExpression(const std::vector<GiNaC::ex>& expressions,
        const std::vector<RobotNode::SharedPtr> joint_nodes, const unsigned int& rows, const unsigned int& cols,
        const JointNameToValueMap& joint_positions, const JointNameToValueMap& joint_velocities,
        const JointNameToValueMap& joint_accelerations) const;

    std::string m_name;
    uint64_t m_id;
    char m_type;
    double m_mass;
    std::vector<double> m_joint_axis;
    SharedPtr m_parent = nullptr;
    std::vector<WeakPtr> m_children;
    std::vector<SharedPtr> m_joint_nodes;
    std::vector<WeakPtr> m_relative_joint_nodes;

    JointSymbol m_joint_symbol_position;
    JointSymbol m_joint_symbol_velocity;
    JointSymbol m_joint_symbol_acceleration;
    GiNaC::lst m_joint_symbols_list;
    GiNaC::lst m_relative_joint_symbols_list;

    GiNaC::ex m_relative_inertia_expression;
    std::vector<GiNaC::ex> m_global_position_expressions;
    std::vector<GiNaC::ex> m_global_velocity_expressions;
    std::vector<GiNaC::ex> m_global_acceleration_expressions;
    std::vector<GiNaC::ex> m_global_rotation_expressions;
    std::vector<GiNaC::ex> m_global_position_jacobian_expressions;
    std::vector<GiNaC::ex> m_global_rotation_jacobian_expressions;
    std::vector<GiNaC::ex> m_dynamical_torque_expressions;
};

#endif // MARCH_STATE_ESTIMATOR__ROBOT_DESCRIPTION__ROBOT_NODE_HPP_