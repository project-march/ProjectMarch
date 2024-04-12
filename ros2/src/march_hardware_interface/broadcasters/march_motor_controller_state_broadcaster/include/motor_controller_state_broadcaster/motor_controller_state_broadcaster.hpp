/// @author George Vegelien - M7

#ifndef MARCH_MOTOR_CONTROLLER_STATE_BROADCASTER__MOTOR_CONTROLLER_STATE_BROADCASTER_HPP_
#define MARCH_MOTOR_CONTROLLER_STATE_BROADCASTER__MOTOR_CONTROLLER_STATE_BROADCASTER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "march_shared_msgs/msg/joint_motor_controller_state.hpp"
#include "march_shared_msgs/msg/motor_controller_states.hpp"
#include "motor_controller_state_broadcaster/motor_controller_semantic_component.hpp"
#include "motor_controller_state_broadcaster/visibility_control.h"
#include "realtime_tools/realtime_publisher.h"

namespace march_motor_controller_state_broadcaster {

using MCsMsg = march_shared_msgs::msg::MotorControllerStates;
using JointMCMsg = march_shared_msgs::msg::JointMotorControllerState;
using MCsRTPublisher = realtime_tools::RealtimePublisher<MCsMsg>;

class MotorControllerStateBroadcaster : public controller_interface::ControllerInterface {
public:
    MOTOR_CONTROLLER_STATE_BROADCASTER_PUBLIC
    MotorControllerStateBroadcaster();

    MOTOR_CONTROLLER_STATE_BROADCASTER_PUBLIC
    controller_interface::return_type init(const std::string& controller_name) override;

    MOTOR_CONTROLLER_STATE_BROADCASTER_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    MOTOR_CONTROLLER_STATE_BROADCASTER_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    MOTOR_CONTROLLER_STATE_BROADCASTER_PUBLIC
    controller_interface::return_type update() override;

    MOTOR_CONTROLLER_STATE_BROADCASTER_PUBLIC
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;

    MOTOR_CONTROLLER_STATE_BROADCASTER_PUBLIC
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    MOTOR_CONTROLLER_STATE_BROADCASTER_PUBLIC
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

private:
    std::map<std::string, std::unique_ptr<MotorControllerSemanticComponent>> possible_components_;
    std::shared_ptr<rclcpp::Logger> logger_;
    rclcpp::Publisher<MCsMsg>::SharedPtr motor_controller_state_publisher_;
    std::unique_ptr<MCsRTPublisher> realtime_motor_controller_publisher_;
};

} // namespace march_motor_controller_state_broadcaster

#endif // MARCH_MOTOR_CONTROLLER_STATE_BROADCASTER__MOTOR_CONTROLLER_STATE_BROADCASTER_HPP_
