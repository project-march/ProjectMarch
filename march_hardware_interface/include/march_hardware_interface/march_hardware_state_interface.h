// Copyright 2019 Project March

#ifndef HARDWARE_INTERFACE_MARCH_HARDWARE_STATE_INTERFACE_H
#define HARDWARE_INTERFACE_MARCH_HARDWARE_STATE_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>

namespace march_hardware_interface
{
class MarchHardwareStateHandle
{
public:
  MarchHardwareStateHandle(
      const std::string& name,      ///< The name of the example
      const double* example    ///< A pointer to an example value
      )
    : name_(name)
    , example_(example)
  {
  }

  std::string getName() const
  {
    return name_;
  }

  const double* getExample() const
  {
    return example_;
  }

private:
  std::string name_;

  const double* example_;
};

class MarchHardwareStateInterface : public hardware_interface::HardwareResourceManager<MarchHardwareStateHandle>
{
};
}

#endif  // HARDWARE_INTERFACE_MARCH_HARDWARE_STATE_INTERFACE_H