/// @author George Vegelien - M7

#ifndef MARCH_PDB_STATE_BROADCASTER__PDB_STATE_BROADCASTER_HPP_
#define MARCH_PDB_STATE_BROADCASTER__PDB_STATE_BROADCASTER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "march_shared_msgs/msg/power_distribution_board_data.h"
#include "pdb_state_broadcaster/pdb_semantic_component.hpp"
#include "pdb_state_broadcaster/visibility_control.h"
#include "realtime_tools/realtime_publisher.h"

namespace march_pdb_state_broadcaster {

using PDB_MSG = march_shared_msgs::msg::PowerDistributionBoardData;
using PdbRTPublisher = realtime_tools::RealtimePublisher<PDB_MSG>;

class PdbStateBroadcaster : public controller_interface::ControllerInterface {
public:
    PDB_STATE_BROADCASTER_PUBLIC
    PdbStateBroadcaster();

    PDB_STATE_BROADCASTER_PUBLIC
    controller_interface::return_type init(const std::string& controller_name) override;

    PDB_STATE_BROADCASTER_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    PDB_STATE_BROADCASTER_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    PDB_STATE_BROADCASTER_PUBLIC
    controller_interface::return_type update() override;

    PDB_STATE_BROADCASTER_PUBLIC
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;

    PDB_STATE_BROADCASTER_PUBLIC
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    PDB_STATE_BROADCASTER_PUBLIC
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

private:
    std::unique_ptr<PdbSemanticComponent> pdb_component_;
    std::shared_ptr<rclcpp::Logger> logger_;
    rclcpp::Publisher<PDB_MSG>::SharedPtr pdb_state_publisher_;
    std::unique_ptr<PdbRTPublisher> realtime_pdb_publisher_;
};

} // namespace march_pdb_state_broadcaster

#endif // MARCH_PDB_STATE_BROADCASTER__PDB_STATE_BROADCASTER_HPP_
