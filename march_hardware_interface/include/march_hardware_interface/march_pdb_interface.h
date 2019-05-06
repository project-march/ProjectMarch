// Copyright 2019 Project March

#ifndef HARDWARE_INTERFACE_MARCH_PDB_STATE_INTERFACE_H
#define HARDWARE_INTERFACE_MARCH_PDB_STATE_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>

namespace march_pdb_state_interface
{
class MarchPDBHandle
{
public:
  MarchPDBHandle(
      const std::string& name,      ///< The name of joint
      const double* pdb_current,    ///< A pointer to the storage of the pdb current value in ??.
      )
    : name_(name)
    , temperature_(pdb_current)
  {
  }

  std::string getName() const
  {
    return name_;
  }

  const double* getPDBCurrent() const
  {
    return pdb_current_;
  }

private:
  std::string name_;

  const double* pdb_current_;
};

class MarchPDBHandleInterface : public hardware_interface::HardwareResourceManager<MarchPDBHandleHandle>
{
};
}

#endif  // HARDWARE_INTERFACE_MARCH_PDB_STATE_INTERFACE_H