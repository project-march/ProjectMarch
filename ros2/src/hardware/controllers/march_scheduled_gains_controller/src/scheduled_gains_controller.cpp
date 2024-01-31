#include "scheduled_gains_controller/scheduled_gains_controller.hpp"


namespace march_scheduled_gains_controller {


// The constructor is called only once when the controller is created. 
// It is not part of the real-time loop, therefore it is safe to allocate memory here.
ScheduledGainsController::ScheduledGainsController() {

}


// The first line usually calls the parent on_init method. Here is the best place to initialize the variables, reserve memory, 
// and most importantly, declare node parameters used by the controller. 
// If everything works fine return controller_interface::return_type::OK or controller_interface::return_type::ERROR otherwise.
controller_interface::return_type ScheduledGainsController::init(const std::string& controller_name) {
    

    return controller_interface::return_type::OK;
}


// Parameters are usually read here, and everything is prepared so that the controller can be started.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ScheduledGainsController::on_configure(
    const rclcpp_lifecycle::State& previous_state) 
{
       
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


// Required interfaces are defined here. There are three options of the interface configuration ALL, INDIVIDUAL, and NONE defined in controller_interface/controller_interface.hpp". 
// ALL and NONE option will ask for access to all available interfaces or none of them. The INDIVIDUAL configuration needs a detailed list of required interface names. 
// Those are usually provided as parameters. The full interface names have structure <joint_name>/<interface_type>.
controller_interface::InterfaceConfiguration ScheduledGainsController::command_interface_configuration() const {
    
    return { controller_interface::interface_configuration_type::NONE };
}

controller_interface::InterfaceConfiguration ScheduledGainsController::state_interface_configuration() const {

    return { controller_interface::interface_configuration_type::INDIVIDUAL };
}

// Checking, and potentially sorting, the interfaces and assigning members’ initial values. 
// This method is part of the real-time loop, therefore avoid any reservation of memory and, in general, keep it as short as possible.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ScheduledGainsController::on_activate(const rclcpp_lifecycle::State& previous_state) {
    

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


// Does the opposite of on_activate. In many cases, this method is empty. This method should also be real-time safe as much as possible.
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ScheduledGainsController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


// The main entry point. The method should be implemented with real-time constraints in mind. 
// When this method is called, the state interfaces have the most recent values from the hardware, and new commands for the hardware should be written into command interfaces.
controller_interface::return_type ScheduledGainsController::update() {
   
    return controller_interface::return_type::OK;
}

}  // namespace march_scheduled_weights_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(march_scheduled_gains_controller::ScheduledGainsController,
    controller_interface::ControllerInterface)