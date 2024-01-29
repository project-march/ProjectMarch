#ifndef FUZZY_WEIGHTS_CONTROLLER__FUZZY_WEIGHTS_CONTROLLER_HPP_
#define FUZZY_WEIGHTS_CONTROLLER__FUZZY_WEIGHTS_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/handle.hpp"

namespace march_fuzzy_weights_controller {

class WeightsController : public controller_interface::ControllerInterface
{
public:
    
    controller_interface::return_type on_init() override


    controller_interface::InterfaceConfiguration command_interface_configuration() const override


    controller_interface::InterfaceConfiguration state_interface_configuration() const override
    

    controller_interface::InterfaceConfiguration on_configure() const override


    controller_interface::InterfaceConfiguration on_activate() override


    controller_interface::InterfaceConfiguration on_deactivate() override
    

    controller_interface::return_type update() override

private:
    // Add any necessary member variables here. This could include handles to the
    // command interfaces, subscribers, etc.
};

}  // namespace march_fuzzy_weights_controller