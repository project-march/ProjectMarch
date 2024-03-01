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
    rosout_publisher_ = node_->create_publisher<rcl_interfaces::msg::Log>("/rosout", 10);
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
    make_rosout_message("Hi");

    return controller_interface::return_type::OK;
}

std::shared_ptr<rcl_interfaces::msg::Log> LogBroadcaster::make_rosout_message(std::string msg)
{
    auto log_msg = std::make_shared<rcl_interfaces::msg::Log>();
    log_msg->stamp = node_->now();
    log_msg->level = rcl_interfaces::msg::Log::ERROR;
    log_msg->name = "hardware_interface_logger_node";
    log_msg->msg = "This log comes from the rosout publisher";
    log_msg->file = "hardware_interface_logger_node.cpp";
    log_msg->function = "rclcpp:spin(node)";
    log_msg->line = 5;

    return log_msg;
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

#include "log_broadcaster.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(march_log_broadcaster::LogBroadcaster, controller_interface::ControllerInterface)