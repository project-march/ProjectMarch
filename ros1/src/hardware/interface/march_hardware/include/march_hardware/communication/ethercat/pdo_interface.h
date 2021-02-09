#ifndef MARCH_HARDWARE_PDO_INTERFACE_H
#define MARCH_HARDWARE_PDO_INTERFACE_H
#include "pdo_types.h"

#include <cstdint>
#include <memory>
#include <utility>

namespace march
{
/**
 * An interface to read and write Process Data Objects (PDOs).
 */
class PdoInterface
{
public:
  virtual void write8(uint16_t slave_index, uint8_t module_index, bit8 value) = 0;
  virtual void write16(uint16_t slave_index, uint8_t module_index, bit16 value) = 0;
  virtual void write32(uint16_t slave_index, uint8_t module_index, bit32 value) = 0;

  virtual bit8 read8(uint16_t slave_index, uint8_t module_index) const = 0;
  virtual bit16 read16(uint16_t slave_index, uint8_t module_index) const = 0;
  virtual bit32 read32(uint16_t slave_index, uint8_t module_index) const = 0;
};

/**
 * Shared pointer to an PdoInterface.
 */
using PdoInterfacePtr = std::shared_ptr<PdoInterface>;

/**
 * An interface to read and write Process Data Objects (PDOs) for a given slave.
 */
class PdoSlaveInterface
{
public:
  PdoSlaveInterface(uint16_t slave_index, PdoInterfacePtr pdo) : slave_index_(slave_index), pdo_(pdo)
  {
  }

  virtual ~PdoSlaveInterface() noexcept = default;

  void write8(uint8_t module_index, bit8 value)
  {
    this->pdo_->write8(this->slave_index_, module_index, value);
  }
  void write16(uint8_t module_index, bit16 value)
  {
    this->pdo_->write16(this->slave_index_, module_index, value);
  }
  void write32(uint8_t module_index, bit32 value)
  {
    this->pdo_->write32(this->slave_index_, module_index, value);
  }

  bit8 read8(uint8_t module_index) const
  {
    return this->pdo_->read8(this->slave_index_, module_index);
  }
  bit16 read16(uint8_t module_index) const
  {
    return this->pdo_->read16(this->slave_index_, module_index);
  }
  bit32 read32(uint8_t module_index) const
  {
    return this->pdo_->read32(this->slave_index_, module_index);
  }

private:
  const uint16_t slave_index_;
  PdoInterfacePtr pdo_;
};

/**
 * An implementation of the PdoInterface using SOEM.
 */
class PdoInterfaceImpl : public PdoInterface
{
public:
  /**
   * Creates a new shared PdoInterfaceImpl.
   * @return Generic PdoInterface shared ptr
   */
  static PdoInterfacePtr create()
  {
    return std::make_shared<PdoInterfaceImpl>();
  }

  void write8(uint16_t slave_index, uint8_t module_index, bit8 value) override;
  void write16(uint16_t slave_index, uint8_t module_index, bit16 value) override;
  void write32(uint16_t slave_index, uint8_t module_index, bit32 value) override;

  bit8 read8(uint16_t slave_index, uint8_t module_index) const override;
  bit16 read16(uint16_t slave_index, uint8_t module_index) const override;
  bit32 read32(uint16_t slave_index, uint8_t module_index) const override;
};
}  // namespace march
#endif  // MARCH_HARDWARE_PDO_INTERFACE_H
