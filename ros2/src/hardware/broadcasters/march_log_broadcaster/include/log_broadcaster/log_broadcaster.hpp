/// @author Martijn Habers - M9

#ifndef MARCH_LOG_BROADCASTER__LOG_BROADCASTER_HPP_
#define MARCH_LOG_BROADCASTER__LOG_BROADCASTER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "log_broadcaster/log_semantic_component.hpp"
#include "log_broadcaster/visibility_control.h"

namespace march_log_broadcaster {

class LogBroadcaster : public controller_interface::ControllerInterface {
public:
    LOG_BROADCASTER_PUBLIC
    LogBroadcaster();

    LOG_BROADCASTER_PUBLIC
    controller_interface::return_type init(const std::string& controller_name) override;

    LOG_BROADCASTER_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    LOG_BROADCASTER_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    LOG_BROADCASTER_PUBLIC
    controller_interface::return_type update() override;

    LOG_BROADCASTER_PUBLIC
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;

    LOG_BROADCASTER_PUBLIC
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    LOG_BROADCASTER_PUBLIC
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

private:
    std::shared_ptr<rclcpp::Logger> logger_;
};

} // namespace march_log_broadcaster

#endif // MARCH_LOG_BROADCASTER__LOG_BROADCASTER_HPP_
