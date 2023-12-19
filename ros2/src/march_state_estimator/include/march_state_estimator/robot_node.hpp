#ifndef MARCH_STATE_ESTIMATOR__ROBOT_NODE_HPP_
#define MARCH_STATE_ESTIMATOR__ROBOT_NODE_HPP_

#define NO_INERTIA_PARAMS 6

class RobotNode
{
public:
    RobotNode();
    ~RobotNode() = default;

private:

    double mass_;
    double inertia_[NO_INERTIA_PARAMS];

};

#endif  // MARCH_STATE_ESTIMATOR__ROBOT_NODE_HPP_