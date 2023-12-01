#ifndef IK_SOLVER__TASK_CLIENT_HPP
#define IK_SOLVER__TASK_CLIENT_HPP

#include <memory>
#include <vector>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "march_shared_msgs/srv/get_task_report.hpp"

class TaskClient : public rclcpp::Node
{
    public:
        TaskClient();
        ~TaskClient();

    private:
        rclcpp::Client<march_shared_msgs::srv::GetTaskReport>::SharedPtr client_;
        void send_request();
        void handle_response();

};

#endif // IK_SOLVER__TASK_CLIENT_HPP