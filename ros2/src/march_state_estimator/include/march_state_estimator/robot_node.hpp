#ifndef MARCH_STATE_ESTIMATOR__ROBOT_NODE_HPP_
#define MARCH_STATE_ESTIMATOR__ROBOT_NODE_HPP_

#include <string>
#include <memory>
#include <vector>

#include "ginac/ginac.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

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
    Eigen::Vector3d getOriginPosition() const;
    Eigen::Matrix3d getOriginRotation() const;
    RobotNode* getParent() const;
    std::vector<RobotNode*> getChildren() const;

    Eigen::Vector3d getGlobalPosition() const;
    Eigen::Matrix3d getGlobalRotation() const;

protected:

    std::string name_;
    uint64_t id_;
    char type_;
    double mass_;
    double length_;
    double inertia_[NO_INERTIA_PARAMS];
    Eigen::Vector3d origin_position_;
    Eigen::Matrix3d origin_rotation_;
    RobotNode* parent_ = nullptr;
    std::vector<RobotNode*> children_;

};

#endif  // MARCH_STATE_ESTIMATOR__ROBOT_NODE_HPP_