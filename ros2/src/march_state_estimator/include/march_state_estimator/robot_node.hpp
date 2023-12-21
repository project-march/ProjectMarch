#ifndef MARCH_STATE_ESTIMATOR__ROBOT_NODE_HPP_
#define MARCH_STATE_ESTIMATOR__ROBOT_NODE_HPP_

#include <string>
#include <memory>
#include <vector>

#include "ginac/ginac.h"
// #include <eigen3/Eigen/Core>
// #include <eigen3/Eigen/Geometry>

#define NO_INERTIA_PARAMS 6

class RobotNode
{
public:
    RobotNode() = default;
    ~RobotNode() = default;

    void setInertia(const std::vector<double> inertia);
    void setParent(RobotNode* parent);
    void addChild(RobotNode* child);

    std::string getName() const;
    uint64_t getId() const;
    uint64_t getId(const std::string & name) const;
    char getType() const;
    double getMass() const;
    double getLength() const;
    std::vector<double> getInertia() const;
    RobotNode* getParent() const;
    std::vector<RobotNode*> getChildren() const;

protected:

    std::string name_;
    uint64_t id_;
    char type_;
    double mass_;
    double length_;
    double inertia_[NO_INERTIA_PARAMS];
    RobotNode* parent_ = nullptr;
    std::vector<RobotNode*> children_;

};

#endif  // MARCH_STATE_ESTIMATOR__ROBOT_NODE_HPP_