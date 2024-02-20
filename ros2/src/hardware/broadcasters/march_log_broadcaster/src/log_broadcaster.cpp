/// @author Martijn Habers - M9

#include "log_broadcaster/log_broadcaster.hpp"

namespace march_log_broadcaster {

LogBroadcaster::LogBroadcaster()
    : logger_(std::make_shared<rclcpp::Logger>(node_->get_logger()))
{
}

controller_interface::return_type LogBroadcaster::init(const std::string& controller_name)
{
    auto ret = ControllerInterface::init(controller_name);
    if (ret != controller_interface::return_type::OK) {
        RCLCPP_FATAL((*logger_), "Log broadcaster init was not 'OK' but was 'ERROR'.");
        return ret;
    }
    return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LogBroadcaster::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_DEBUG((*logger_), "March Log broadcaster configuring. Previous state = %s", previous_state.label().c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration LogBroadcaster::command_interface_configuration() const
{
    return { controller_interface::interface_configuration_type::NONE };
}

controller_interface::InterfaceConfiguration LogBroadcaster::state_interface_configuration() const
{
    return { controller_interface::interface_configuration_type::ALL };
}

controller_interface::return_type LogBroadcaster::update()
{
    return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LogBroadcaster::on_activate(
    const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_DEBUG((*logger_), "March Log broadcaster activating. Previous state = %s", previous_state.label().c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LogBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_DEBUG((*logger_), "March Log broadcaster deactivating. Previous state = %s", previous_state.label().c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
} // namespace march_log_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(march_log_broadcaster::LogBroadcaster, controller_interface::ControllerInterface)