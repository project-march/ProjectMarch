// Copyright 2019 Project MarchMarchPdbStateHandlen PDO Map (total bits 272, only 2

#ifndef HARDWARE_INTERFACE_MARCH_PDB_STATE_INTERFACE_H
#define HARDWARE_INTERFACE_MARCH_PDB_STATE_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>

namespace march_hardware_interface
{
class MarchPdbStateHandle
{
public:
  MarchPdbStateHandle(
      const std::string& name,      ///< The name of joint
      const double* pdb_current    ///< A pointer to the storage of the pdb current value in ??.
      )
    : name_(name)
    , pdb_current_(pdb_current)
  {
  }

  std::string getName() const
  {
    return name_;
  }

  const double* getPdbCurrent() const
  {
    return pdb_current_;
  }

private:
  std::string name_;

  const double* pdb_current_;
};

class MarchPdbStateInterface : public hardware_interface::HardwareResourceManager<MarchPdbStateHandle>
{
};
}

#endif  // HARDWARE_INTERFACE_MARCH_PDB_STATE_INTERFACE_H