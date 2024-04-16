/// @author George Vegelien - M7

#include "pdb_state_broadcaster/pdb_state_broadcaster.hpp"

namespace march_pdb_state_broadcaster {

PdbStateBroadcaster::PdbStateBroadcaster()
    : logger_(std::make_shared<rclcpp::Logger>(rclcpp::get_logger("pdb_state_broadcaster")))
{
}

controller_interface::return_type PdbStateBroadcaster::init(const std::string& controller_name)
{
    auto ret = ControllerInterface::init(controller_name);
    if (ret != controller_interface::return_type::OK) {
        RCLCPP_FATAL((*logger_), "PDB State broadcaster init was not 'OK' but was 'ERROR'.");
        return ret;
    }
    return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn PdbStateBroadcaster::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_DEBUG((*logger_), "March PDB broadcaster configuring. Previous state = %s", previous_state.label().c_str());
    pdb_component_ = std::make_unique<PdbSemanticComponent>();
    try {
        // register pdb data publisher
        pdb_state_publisher_ = node_->create_publisher<PDB_MSG>("/march/pdb_data", rclcpp::SystemDefaultsQoS());
        realtime_pdb_publisher_ = std::make_unique<PdbRTPublisher>(pdb_state_publisher_);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(
            (*logger_), "Exception thrown during publisher creation at configure stage with message : %s \n", e.what());
        return CallbackReturn::ERROR;
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration PdbStateBroadcaster::command_interface_configuration() const
{
    return { controller_interface::interface_configuration_type::NONE };
}

controller_interface::InterfaceConfiguration PdbStateBroadcaster::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    state_interfaces_config.names = pdb_component_->get_state_interface_names();
    return state_interfaces_config;
}

controller_interface::return_type PdbStateBroadcaster::update()
{

    if (realtime_pdb_publisher_ && realtime_pdb_publisher_->trylock()) {
        realtime_pdb_publisher_->msg_.header.stamp = node_->now();
        pdb_component_->get_values_as_message(realtime_pdb_publisher_->msg_);
        realtime_pdb_publisher_->unlockAndPublish();
    }
    return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn PdbStateBroadcaster::on_activate(
    const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_DEBUG((*logger_), "March PDB broadcaster activating. Previous state = %s", previous_state.label().c_str());
    pdb_component_->assign_loaned_state_interfaces(state_interfaces_);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn PdbStateBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_DEBUG((*logger_), "March PDB broadcaster deactivating. Previous state = %s", previous_state.label().c_str());
    pdb_component_->release_interfaces();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
} // namespace march_pdb_state_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(march_pdb_state_broadcaster::PdbStateBroadcaster, controller_interface::ControllerInterface)
