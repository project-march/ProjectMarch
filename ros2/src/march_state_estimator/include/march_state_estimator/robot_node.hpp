#ifndef MARCH_STATE_ESTIMATOR__ROBOT_NODE_HPP_
#define MARCH_STATE_ESTIMATOR__ROBOT_NODE_HPP_

#include <string>
#include <memory>
#include <vector>

// #include <eigen3/Eigen/Core>
// #include <eigen3/Eigen/Geometry>

#define NO_INERTIA_PARAMS 6

class RobotNode
{
public:
    RobotNode() = default;
    ~RobotNode() = default;

protected:

    std::string name_;
    double mass_;
    double length_;
    double inertia_[NO_INERTIA_PARAMS];
    std::shared_ptr<RobotNode> parent_;
    std::shared_ptr<RobotNode> child_;

};

#endif  // MARCH_STATE_ESTIMATOR__ROBOT_NODE_HPP_