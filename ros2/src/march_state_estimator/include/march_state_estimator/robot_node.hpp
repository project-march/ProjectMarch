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
    Eigen::Vector3d getGlobalPosition() const;
    Eigen::Matrix3d getGlobalRotation() const;
    RobotNode* getParent() const;
    std::vector<RobotNode*> getChildren() const;

    void expressKinematics();
    void expressDynamics();

protected:

    std::vector<GiNaC::symbol> getJointAngles() const;
    GiNaC::matrix expressGlobalPosition() const;
    GiNaC::matrix expressGlobalRotation() const;

    GiNaC::matrix utilConvertEigenToGiNaC(const Eigen::MatrixXd & matrix) const;
    Eigen::Matrix3d utilConvertGiNaCToEigen(const GiNaC::matrix & matrix) const;

    std::string name_;
    uint64_t id_;
    char type_;
    double mass_;
    double length_;
    double inertia_[NO_INERTIA_PARAMS];
    RobotNode* parent_ = nullptr;
    std::vector<RobotNode*> children_;

    GiNaC::symbol joint_angle_;
    GiNaC::matrix global_position_vector_;
    GiNaC::matrix global_rotation_matrix_;
    GiNaC::matrix origin_position_vector_;
    GiNaC::matrix origin_rotation_matrix_;
    std::vector<GiNaC::symbol> joint_angles_;

};

#endif  // MARCH_STATE_ESTIMATOR__ROBOT_NODE_HPP_