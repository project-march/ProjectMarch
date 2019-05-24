// Copyright 2019 Project March.

#ifndef HARDWARE_INTERFACE_MARCH_PDB_STATE_INTERFACE_H
#define HARDWARE_INTERFACE_MARCH_PDB_STATE_INTERFACE_H

#include <string>

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <march_hardware/PowerDistributionBoard.h>
#include <march_hardware_interface/PowerNetOnOffCommand.h>

namespace march_hardware_interface {
class MarchPdbStateHandle {
public:
  MarchPdbStateHandle(
      /// Pointers to the storage location
      const std::string &name,
      const march4cpp::PowerDistributionBoard *powerDistributionBoard,
      bool *master_shutdown_allowed_command,
      bool *trigger_emergency_switch_command,
      PowerNetOnOffCommand *power_net_on_off_command)
      : name_(name), powerDistributionBoard_(powerDistributionBoard),
        master_shutdown_allowed_command_(master_shutdown_allowed_command),
        trigger_emergency_switch_command_(trigger_emergency_switch_command),
        power_net_on_off_command_(power_net_on_off_command){}

  MarchPdbStateHandle() {}

  std::string getName() const { return name_; }

  march4cpp::PowerDistributionBoard *getPowerDistributionBoard() {
    return const_cast<march4cpp::PowerDistributionBoard *>(
        powerDistributionBoard_);
  }

  void setMasterShutdownAllowed(bool is_allowed) {
    assert(master_shutdown_allowed_command_);
    ROS_INFO("setMasterShutdownAllowed %d", is_allowed);
    *master_shutdown_allowed_command_ = is_allowed;
  }

  void triggerEmergencySwitch(bool trigger) {
    assert(trigger_emergency_switch_command_);
    ROS_INFO("triggerEmergencySwitch %d", trigger);
    *trigger_emergency_switch_command_ = trigger;
  }

  void turnNetOnOrOff(PowerNetType type, bool on_or_off, int net_number){
    assert(power_net_on_off_command_);
    PowerNetOnOffCommand power_net_on_off_command(type, on_or_off, net_number);
    ROS_INFO_STREAM("turnNetOnOrOff: " << power_net_on_off_command);
    *power_net_on_off_command_ = power_net_on_off_command;
  }

private:
  std::string name_;
  bool *master_shutdown_allowed_command_;
  bool *trigger_emergency_switch_command_;
  PowerNetOnOffCommand *power_net_on_off_command_;
  const march4cpp::PowerDistributionBoard *powerDistributionBoard_;
};

class MarchPdbStateInterface
    : public hardware_interface::HardwareResourceManager<MarchPdbStateHandle> {
};
}

#endif // HARDWARE_INTERFACE_MARCH_PDB_STATE_INTERFACE_H