#ifndef MARCH_SCHEDULED_GAINS_CONTROLLER__SCHEDULED_GAINS_CONTROLLER_HPP_
#define MARCH_SCHEDULED_GAINS_CONTROLLER__SCHEDULED_GAINS_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/handle.hpp"
#include "scheduled_gains_controller/visibility_control.h"
#include "realtime_tools/realtime_publisher.h"

namespace march_scheduled_gains_controller {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
class ScheduledGainsController : public controller_interface::ControllerInterface
{
public:
    
    SCHEDULED_GAINS_CONTROLLER_PUBLIC
    ScheduledGainsController();

    SCHEDULED_GAINS_CONTROLLER_PUBLIC
    controller_interface::return_type init(const std::string& controller_name) override;

    SCHEDULED_GAINS_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    SCHEDULED_GAINS_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    
    SCHEDULED_GAINS_CONTROLLER_PUBLIC
    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override; 

    SCHEDULED_GAINS_CONTROLLER_PUBLIC
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override; 

    SCHEDULED_GAINS_CONTROLLER_PUBLIC
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override; 
    
    SCHEDULED_GAINS_CONTROLLER_PUBLIC
    controller_interface::return_type update() override;

private:
    // Add any necessary member variables here. This could include handles to the
    // command interfaces, subscribers, etc.
};

}  // namespace march_scheduled_gains_controller

#endif // MARCH_SCHEDULED_GAINS_CONTROLLER__SCHEDULED_GAINS_CONTROLLER_HPP_