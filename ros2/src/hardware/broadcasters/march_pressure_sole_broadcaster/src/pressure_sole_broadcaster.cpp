/// @author Marco Bak - M8

#include "pressure_sole_broadcaster/pressure_sole_broadcaster.hpp"

namespace march_pressure_sole_broadcaster {

PressureSoleBroadcaster::PressureSoleBroadcaster()
    : logger_(std::make_shared<rclcpp::Logger>(rclcpp::get_logger("pressure_sole_broadcaster")))
{
}

controller_interface::return_type PressureSoleBroadcaster::init(const std::string& controller_name)
{
    auto ret = ControllerInterface::init(controller_name);
    if (ret != controller_interface::return_type::OK) {
        RCLCPP_FATAL((*logger_), "Pressure sole broadcaster init was not 'OK' but was 'ERROR'.");
        return ret;
    }
    return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration PressureSoleBroadcaster::command_interface_configuration() const
{
    return { controller_interface::interface_configuration_type::NONE };
}

controller_interface::InterfaceConfiguration PressureSoleBroadcaster::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    state_interfaces_config.names = pressure_sole_component->get_state_interface_names();
    return state_interfaces_config;
}

controller_interface::return_type PressureSoleBroadcaster::update()
{

    if (realtime_pressure_sole_publisher_ && realtime_pressure_sole_publisher_->trylock()) {
        realtime_pressure_sole_publisher_->msg_.header.stamp = node_->now();
        pressure_sole_component->get_values_as_message(realtime_pressure_sole_publisher_->msg_);
        realtime_pressure_sole_publisher_->unlockAndPublish();
    }
    return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn PressureSoleBroadcaster::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_DEBUG(
        (*logger_), "March Pressure sole broadcaster configuring. Previous state = %s", previous_state.label().c_str());
    pressure_sole_component = std::make_unique<PressureSoleSemanticComponent>();
    try {
        // register pressure sole data publisher
        pressure_sole_publisher_
            = node_->create_publisher<PRESSURE_SOLE_MSG>("/march/pressure_sole_data", rclcpp::SystemDefaultsQoS());
        realtime_pressure_sole_publisher_ = std::make_unique<PressureSoleRTPublisher>(pressure_sole_publisher_);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(
            (*logger_), "Exception thrown during publisher creation at configure stage with message : %s \n", e.what());
        return CallbackReturn::ERROR;
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn PressureSoleBroadcaster::on_activate(
    const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_DEBUG(
        (*logger_), "March pressure sole broadcaster activating. Previous state = %s", previous_state.label().c_str());
    pressure_sole_component->assign_loaned_state_interfaces(state_interfaces_);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn PressureSoleBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_DEBUG((*logger_), "March Pressure sole broadcaster deactivating. Previous state = %s",
        previous_state.label().c_str());
    pressure_sole_component->release_interfaces();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
} // namespace march_pressure_sole_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    march_pressure_sole_broadcaster::PressureSoleBroadcaster, controller_interface::ControllerInterface)
