/// @author George Vegelien - M7

#include "motor_controller_state_broadcaster/motor_controller_state_broadcaster.hpp"

namespace march_motor_controller_state_broadcaster {

MotorControllerStateBroadcaster::MotorControllerStateBroadcaster()
    : logger_(std::make_shared<rclcpp::Logger>(rclcpp::get_logger("motor_controller_state_broadcaster")))
{
}

controller_interface::return_type MotorControllerStateBroadcaster::init(const std::string& controller_name)
{
    auto ret = ControllerInterface::init(controller_name);
    if (ret != controller_interface::return_type::OK) {
        RCLCPP_FATAL((*logger_), "Motor Controller State broadcaster init was not 'OK' but was 'ERROR'.");
        return ret;
    }
    auto_declare<std::vector<std::string>>("possible_joints", {});

    return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MotorControllerStateBroadcaster::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_DEBUG((*logger_), "March PDB broadcaster configuring. Previous state = %s", previous_state.label().c_str());
    std::vector<std::string> possible_joints_names = get_node()->get_parameter("possible_joints").as_string_array();
    if (possible_joints_names.empty()) {
        RCLCPP_FATAL((*logger_),
            "No possible joints defined in the control yaml. "
            "This broadcaster won't publish anything.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    for (const auto& name : possible_joints_names) {
        possible_components_[name] = std::make_unique<MotorControllerSemanticComponent>(name);
    }

    try {
        // register motor controller data publisher
        motor_controller_state_publisher_
            = node_->create_publisher<MCsMsg>("/march/motor_controller_states", rclcpp::SystemDefaultsQoS());
        realtime_motor_controller_publisher_ = std::make_unique<MCsRTPublisher>(motor_controller_state_publisher_);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(
            (*logger_), "Exception thrown during publisher creation at configure stage with message : %s \n", e.what());
        return CallbackReturn::ERROR;
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration MotorControllerStateBroadcaster::command_interface_configuration() const
{
    return { controller_interface::interface_configuration_type::NONE };
}

controller_interface::InterfaceConfiguration MotorControllerStateBroadcaster::state_interface_configuration() const
{
    return { controller_interface::interface_configuration_type::ALL };
}

controller_interface::return_type MotorControllerStateBroadcaster::update()
{
    if (realtime_motor_controller_publisher_ && realtime_motor_controller_publisher_->trylock()) {
        realtime_motor_controller_publisher_->msg_.header.stamp = node_->now();
        int i = 0;
        for (const auto& possible_joint : possible_components_) {
            JointMCMsg* joint_msg = &realtime_motor_controller_publisher_->msg_.motor_controller_states[i];
            possible_joint.second->get_values_as_message(*joint_msg);
            i++;
        }
        realtime_motor_controller_publisher_->unlockAndPublish();
    }
    return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MotorControllerStateBroadcaster::on_activate(
    const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_DEBUG((*logger_), "March PDB broadcaster activating. Previous state = %s", previous_state.label().c_str());

    // Get a unique set of joint names.
    std::set<std::string> joint_names;
    for (auto si = state_interfaces_.crbegin(); si != state_interfaces_.crend(); si++) {
        RCLCPP_DEBUG((*logger_), "Got state interface %s", si->get_full_name().c_str());
        joint_names.emplace(si->get_name());
    }

    // Get all joint names that are also listed in 'possible_joints' in the control yaml,
    // and assign the loaned interface to the component.
    std::vector<std::string> not_used_possible_joints;
    for (const auto& possible_joint : possible_components_) {
        if (joint_names.find(possible_joint.first) != joint_names.end()) {
            RCLCPP_INFO((*logger_), "Broadcasting motor controller states for: %s", possible_joint.first.c_str());
            possible_joint.second->assign_loaned_state_interfaces(state_interfaces_);
        } else {
            not_used_possible_joints.emplace_back(possible_joint.first);
        }
    }

    // Delete all not used, possible components.
    for (const auto& not_used_joint_name : not_used_possible_joints) {
        RCLCPP_WARN((*logger_), "Not broadcasting for joint names", not_used_joint_name.c_str());
        possible_components_.erase(not_used_joint_name);
    }
    realtime_motor_controller_publisher_->msg_.motor_controller_states.resize(
        possible_components_.size(), JointMCMsg());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MotorControllerStateBroadcaster::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_DEBUG((*logger_), "March PDB broadcaster deactivating. Previous state = %s", previous_state.label().c_str());
    for (const auto& possible_joint : possible_components_) {
        possible_joint.second->release_interfaces();
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
} // namespace march_motor_controller_state_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(march_motor_controller_state_broadcaster::MotorControllerStateBroadcaster,
    controller_interface::ControllerInterface)
