// Copyright 2019 Project MarchMarchPdbStateHandlen PDO Map (total bits 272,
// only 2

#ifndef HARDWARE_INTERFACE_MARCH_PDB_STATE_INTERFACE_H
#define HARDWARE_INTERFACE_MARCH_PDB_STATE_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <march_hardware/PowerDistributionBoard.h>
#include <string>

namespace march_hardware_interface {
class MarchPdbStateHandle {
public:
  MarchPdbStateHandle(
      /// Pointers to the storage location
      const std::string &name,
      const march4cpp::PowerDistributionBoard *powerDistributionBoard,
      bool* master_shutdown_allowed_command,
      bool* trigger_emergency_switch_command)
      : name_(name), powerDistributionBoard_(powerDistributionBoard),
        master_shutdown_allowed_command_(master_shutdown_allowed_command), trigger_emergency_switch_command_(trigger_emergency_switch_command) {}

  std::string getName() const { return name_; }

  march4cpp::PowerDistributionBoard *getPowerDistributionBoard() {
    return const_cast<march4cpp::PowerDistributionBoard *>(
        powerDistributionBoard_);
  }

  void setMasterShutdownAllowed(bool isAllowed) {
    assert(master_shutdown_allowed_command_);
    *master_shutdown_allowed_command_ = isAllowed;
  }

  void triggerEmergencySwitch(bool trigger) {
    assert(trigger_emergency_switch_command_);
    *trigger_emergency_switch_command_ = trigger;
  }

private:
  std::string name_;
  bool *master_shutdown_allowed_command_;
  bool *trigger_emergency_switch_command_;
  const march4cpp::PowerDistributionBoard *powerDistributionBoard_;
};

class MarchPdbStateInterface
    : public hardware_interface::HardwareResourceManager<MarchPdbStateHandle> {
};
}

#endif // HARDWARE_INTERFACE_MARCH_PDB_STATE_INTERFACE_H