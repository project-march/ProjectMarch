#ifndef IK_SOLVER__TASK_CLIENT_HPP
#define IK_SOLVER__TASK_CLIENT_HPP

#include <functional>
#include <memory>
#include <vector>

#include "march_shared_msgs/srv/get_task_report.hpp"
#include "rclcpp/rclcpp.hpp"

class TaskClient : public rclcpp::Node {
public:
    TaskClient();
    ~TaskClient();

private:
    rclcpp::Client<march_shared_msgs::srv::GetTaskReport>::SharedPtr client_;
    void send_request();
    void handle_response();
};

#endif // IK_SOLVER__TASK_CLIENT_HPP