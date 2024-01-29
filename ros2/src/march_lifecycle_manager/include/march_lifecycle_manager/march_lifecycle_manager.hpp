#ifndef LIFECYCLEMANAGER_HPP
#define LIFECYCLEMANAGER_HPP

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include <chrono>
#include <memory>
#include <future>
#include <thread>
#include <string>

class LifecycleManager : public rclcpp::Node
{
public:
    explicit LifecycleManager();
    ~LifecycleManager() = default;
    void init();
    unsigned int get_state(std::chrono::seconds time_out);


private:
    // which node to handle
    static constexpr char const * lifecycle_node = "lc_talker";
    // Every lifecycle node has various services
    // attached to it. By convention, we use the format of
    // <node name>/<service name>.
    // In this demo, we use get_state and change_state
    // and thus the two service topics are:
    // lc_talker/get_state
    // lc_talker/change_state
    static constexpr char const * node_get_state_topic = "lc_talker/get_state";
    static constexpr char const * node_change_state_topic = "lc_talker/change_state";
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;

};

#endif // LIFECYCLEMANAGER_HPP
