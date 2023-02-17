/// @author Marco Bak - M8
#ifndef BUILD_MARCH_PRESSURE_SOLE_BROADCASTER_HPP
#define BUILD_MARCH_PRESSURE_SOLE_BROADCASTER_HPP

#include "controller_interface/controller_interface.hpp"
#include "pressure_sole_broadcaster/pressure_sole_semantic_component.hpp"
#include "pressure_sole_broadcaster/visibilitiy_control.h"
#include <realtime_tools/realtime_publisher.h>

namespace march_pressure_sole_broadcaster {

using PRESSURE_SOLE_MSG = march_shared_msgs::msg::PressureSolesData;
using PressureSoleRTPublisher = realtime_tools::RealtimePublisher<PRESSURE_SOLE_MSG>;

class PressureSoleBroadcaster : public controller_interface::ControllerInterface {
public:
    PRESSURE_SOLE_BROADCASTER_PUBLIC
    PressureSoleBroadcaster();

    PRESSURE_SOLE_BROADCASTER_PUBLIC
    controller_interface::return_type init(const std::string& controller_name) override;

    PRESSURE_SOLE_BROADCASTER_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    PRESSURE_SOLE_BROADCASTER_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    PRESSURE_SOLE_BROADCASTER_PUBLIC
    controller_interface::return_type update() override;

    PRESSURE_SOLE_BROADCASTER_PUBLIC
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;

    PRESSURE_SOLE_BROADCASTER_PUBLIC
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    PRESSURE_SOLE_BROADCASTER_PUBLIC
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

private:
    std::unique_ptr<PressureSoleSemanticComponent> pressure_sole_component;
    std::shared_ptr<rclcpp::Logger> logger_;
    rclcpp::Publisher<PRESSURE_SOLE_MSG>::SharedPtr pressure_sole_publisher_;
    std::unique_ptr<PressureSoleRTPublisher> realtime_pressure_sole_publisher_;
};
} // namespace march_pressure_sole_broadcaster
#endif // BUILD_MARCH_PRESSURE_SOLE_BROADCASTER_HPP
