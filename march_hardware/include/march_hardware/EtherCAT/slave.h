// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_SLAVE_H
#define MARCH_HARDWARE_SLAVE_H
#include "pdo_interface.h"
#include "sdo_interface.h"
#include "march_hardware/error/hardware_exception.h"

#include <ros/ros.h>

namespace march
{
class Slave : public PdoSlaveInterface
{
public:
  Slave(uint16_t slave_index, PdoInterface& pdo_interface, SdoInterface& sdo_interface)
    : PdoSlaveInterface(slave_index, pdo_interface), slave_index_(slave_index), sdo_interface_(sdo_interface)
  {
    if (this->slave_index_ < 1)
    {
      throw error::HardwareException(error::ErrorType::INVALID_SLAVE_INDEX, "Slave index %d is smaller than 1",
                                     this->slave_index_);
    }
  };

  virtual ~Slave() noexcept = default;

  uint16_t getSlaveIndex() const
  {
    return this->slave_index_;
  }

  bool initSdo(int cycle_time)
  {
    SdoSlaveInterface sdo_slave_interface(this->slave_index_, this->sdo_interface_);
    return this->initSdo(sdo_slave_interface, cycle_time);
  }

  void reset()
  {
    SdoSlaveInterface sdo_slave_interface(this->slave_index_, this->sdo_interface_);
    this->reset(sdo_slave_interface);
  }

protected:
  virtual bool initSdo(SdoSlaveInterface& /* sdo */, int /* cycle_time */)
  {
    return false;
  }

  virtual void reset(SdoSlaveInterface& /* sdo */)
  {
  }

private:
  const uint16_t slave_index_;
  SdoInterface& sdo_interface_;
};
}  // namespace march

#endif  // MARCH_HARDWARE_SLAVE_H
