// Copyright 2019 Project MarchMarchPdbStateHandlen PDO Map (total bits 272, only 2

#ifndef HARDWARE_INTERFACE_MARCH_PDB_STATE_INTERFACE_H
#define HARDWARE_INTERFACE_MARCH_PDB_STATE_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <march_hardware/PowerDistributionBoard.h>
#include <string>

namespace march_hardware_interface
{
class MarchPdbStateHandle
{
public:
  MarchPdbStateHandle(
      /// Pointers to the storage location
      const std::string& name, const march4cpp::PowerDistributionBoard* powerDistributionBoard)
    : name_(name), powerDistributionBoard_(powerDistributionBoard)
  {
  }

  std::string getName() const
  {
    return name_;
  }

  march4cpp::PowerDistributionBoard* getPowerDistributionBoard()
  {
    return const_cast<march4cpp::PowerDistributionBoard*>(powerDistributionBoard_);
  }

private:
  std::string name_;
  const march4cpp::PowerDistributionBoard* powerDistributionBoard_;
};

class MarchPdbStateInterface : public hardware_interface::HardwareResourceManager<MarchPdbStateHandle>
{
};
}

#endif  // HARDWARE_INTERFACE_MARCH_PDB_STATE_INTERFACE_H