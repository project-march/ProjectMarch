#ifndef IK_SOLVER__CURRENT_JOINT_POSITIONS_SERVER_HPP_
#define IK_SOLVER__CURRENT_JOINT_POSITIONS_SERVER_HPP_

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "march_shared_msgs/srv/get_current_joint_positions.hpp"
#include "rclcpp/rclcpp.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

class CurrentJointPositionServer : public rclcpp::Node {
public:
    CurrentJointPositionServer();
    ~CurrentJointPositionServer();

private:
    rclcpp::Service<march_shared_msgs::srv::GetCurrentJointPositions>::SharedPtr service_;
    void handle_request(const std::shared_ptr<march_shared_msgs::srv::GetCurrentJointPositions::Request> request,
        std::shared_ptr<march_shared_msgs::srv::GetCurrentJointPositions::Response> response);
};

#endif // IK_SOLVER__CURRENT_JOINT_POSITIONS_SERVER_HPP_