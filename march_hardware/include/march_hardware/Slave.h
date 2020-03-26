// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_SLAVE_H
#define MARCH_HARDWARE_SLAVE_H
#include "march_hardware/error/hardware_exception.h"

#include <ros/ros.h>

namespace march
{
class Slave
{
protected:
  int slaveIndex;

public:
  explicit Slave(int slaveIndex)
  {
    if (slaveIndex < 1)
    {
      throw error::HardwareException(error::ErrorType::INVALID_SLAVE_INDEX, "Slave index %d is smaller than 1",
                                     slaveIndex);
    }
    this->slaveIndex = slaveIndex;
  };

  Slave()
  {
    slaveIndex = -1;
  };

  virtual ~Slave() noexcept = default;

  virtual void writeInitialSDOs(int /* cycle_time */)
  {
  }

  int getSlaveIndex()
  {
    return this->slaveIndex;
  }
};
}  // namespace march

#endif  // MARCH_HARDWARE_SLAVE_H
