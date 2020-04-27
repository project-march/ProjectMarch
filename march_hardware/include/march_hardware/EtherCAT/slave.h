// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_SLAVE_H
#define MARCH_HARDWARE_SLAVE_H
#include "pdo_interface.h"
#include "sdo_interface.h"
#include "march_hardware/error/hardware_exception.h"

#include <ros/ros.h>

namespace march
{
class Slave : public PdoInterface
{
public:
  explicit Slave(int slave_index, PdoInterface& pdo_interface, SdoInterface& sdo_interface)
    : slave_index_(slave_index)
    , pdo_interface_(pdo_interface)
    , sdo_interface_(sdo_interface)
  {
    if (this->slave_index_ < 1)
    {
      throw error::HardwareException(error::ErrorType::INVALID_SLAVE_INDEX, "Slave index %d is smaller than 1",
                                     this->slave_index_);
    }
  };

  virtual ~Slave() noexcept = default;

  int getSlaveIndex() const
  {
    return this->slave_index_;
  }

  void initSdo(int cycle_time)
  {
    this->initSdo(this->sdo_interface_, cycle_time);
  }

protected:
  virtual void initSdo(SdoInterface& /* sdo */, int /* cycle_time */)
  {
  }

  void write8(uint16_t slave_index, uint8_t module_index, bit8 value) override
  {
    this->pdo_interface_.write8(slave_index, module_index, value);
  }
  void write16(uint16_t slave_index, uint8_t module_index, bit16 value) override
  {
    this->pdo_interface_.write16(slave_index, module_index, value);
  }
  void write32(uint16_t slave_index, uint8_t module_index, bit32 value) override
  {
    this->pdo_interface_.write32(slave_index, module_index, value);
  }

  bit8 read8(uint16_t slave_index, uint8_t module_index) const override
  {
    return this->pdo_interface_.read8(slave_index, module_index);
  }
  bit16 read16(uint16_t slave_index, uint8_t module_index) const override
  {
    return this->pdo_interface_.read16(slave_index, module_index);
  }
  bit32 read32(uint16_t slave_index, uint8_t module_index) const override
  {
    return this->pdo_interface_.read32(slave_index, module_index);
  }

private:
  const int slave_index_;
  PdoInterface& pdo_interface_;
  SdoInterface& sdo_interface_;
};
}  // namespace march

#endif  // MARCH_HARDWARE_SLAVE_H
