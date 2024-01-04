#ifndef MARCH_STATE_ESTIMATOR__ROBOT_NODE_HPP_
#define MARCH_STATE_ESTIMATOR__ROBOT_NODE_HPP_

#include <string>
#include <memory>
#include <vector>

#include "ginac/ginac.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#define WORKSPACE_DIM 3
#define NO_INERTIA_PARAMS 6

class RobotNode
{
public:
    RobotNode() = default;
    ~RobotNode() = default;

    void setInertia(const std::vector<double> inertia);
    void setParent(RobotNode* parent);
    void addChild(RobotNode* child);
    void setOriginPosition(const Eigen::Vector3d & position);
    void setOriginRotation(const Eigen::Matrix3d & rotation);

    std::string getName() const;
    uint64_t getId() const;
    uint64_t getId(const std::string & name) const;
    char getType() const;
    double getMass() const;
    double getLength() const;
    std::vector<double> getInertia() const;
    GiNaC::matrix getOriginPosition() const;
    GiNaC::matrix getOriginRotation() const;
    GiNaC::symbol getJointAngle() const;
    Eigen::Vector3d getGlobalPosition(std::vector<std::string> joint_names, std::vector<double> joint_angles) const;
    Eigen::Matrix3d getGlobalRotation() const;
    Eigen::MatrixXd getGlobalPositionJacobian(std::vector<std::string> joint_names, std::vector<double> joint_angles) const;
    Eigen::MatrixXd getGlobalRotationJacobian(std::vector<std::string> joint_names, std::vector<double> joint_angles) const;
    RobotNode* getParent() const;
    std::vector<RobotNode*> getChildren() const;
    std::vector<std::string> getJointNames() const;

    void expressKinematics();
    void expressDynamics();

protected:

    std::vector<GiNaC::symbol> getJointAngles() const;
    std::vector<RobotNode*> getJointNodes() const;
    GiNaC::matrix expressGlobalPosition() const;
    GiNaC::matrix expressGlobalRotation() const;
    GiNaC::matrix expressGlobalPositionJacobian() const;
    GiNaC::matrix expressGlobalRotationJacobian() const;

    GiNaC::matrix utilConvertEigenToGiNaC(const Eigen::MatrixXd & matrix) const;
    Eigen::Matrix3d utilConvertGiNaCToEigen(const GiNaC::matrix & matrix) const;

    std::string m_name;
    uint64_t m_id;
    char m_type;
    double m_mass;
    double m_length;
    double m_inertia[NO_INERTIA_PARAMS];
    RobotNode* m_parent = nullptr;
    std::vector<RobotNode*> m_children;
    std::vector<RobotNode*> m_joint_nodes;

    GiNaC::symbol m_joint_angle;
    GiNaC::matrix m_global_position_vector;
    GiNaC::matrix m_global_rotation_matrix;
    GiNaC::matrix m_origin_position_vector;
    GiNaC::matrix m_origin_rotation_matrix;
    GiNaC::matrix m_global_position_jacobian_matrix;
    GiNaC::matrix m_global_rotation_jacobian_matrix;
    std::vector<GiNaC::symbol> m_joint_angles;

};

#endif  // MARCH_STATE_ESTIMATOR__ROBOT_NODE_HPP_