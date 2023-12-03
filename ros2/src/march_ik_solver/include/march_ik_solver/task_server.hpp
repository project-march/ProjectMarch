#ifndef IK_SOLVER__TASK_SERVER_HPP
#define IK_SOLVER__TASK_SERVER_HPP

#include <memory>
#include <vector>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "march_shared_msgs/srv/get_task_report.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

class TaskServer : public rclcpp::Node
{
    public:
        TaskServer();
        ~TaskServer();

    private:
        rclcpp::Service<march_shared_msgs::srv::GetTaskReport>::SharedPtr service_;
        void handle_request(const std::shared_ptr<march_shared_msgs::srv::GetTaskReport::Request> request,
                std::shared_ptr<march_shared_msgs::srv::GetTaskReport::Response> response);

        // TODO: Load Task from YAML file
};

#endif // IK_SOLVER__TASK_SERVER_HPP