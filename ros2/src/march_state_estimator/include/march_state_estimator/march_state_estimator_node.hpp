#ifndef MARCH_STATE_ESTIMATOR__MARCH_STATE_ESTIMATOR_NODE_HPP_
#define MARCH_STATE_ESTIMATOR__MARCH_STATE_ESTIMATOR_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "march_state_estimator/robot_description.hpp"

class MarchStateEstimatorNode : public rclcpp::Node
{
public:
    MarchStateEstimatorNode();

private:
    std::shared_ptr<RobotDescription> robot_description_;

};

#endif // MARCH_STATE_ESTIMATOR__MARCH_STATE_ESTIMATOR_NODE_HPP_