#ifndef MARCH_HARDWARE_PDO_INTERFACE_H
#define MARCH_HARDWARE_PDO_INTERFACE_H
#include "pdo_types.h"

#include <cstdint>

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

class PdoInterfaceImpl : public PdoInterface
{
public:
  void write8(uint16_t slave_index, uint8_t module_index, bit8 value) override;
  void write16(uint16_t slave_index, uint8_t module_index, bit16 value) override;
  void write32(uint16_t slave_index, uint8_t module_index, bit32 value) override;

  bit8 read8(uint16_t slave_index, uint8_t module_index) const override;
  bit16 read16(uint16_t slave_index, uint8_t module_index) const override;
  bit32 read32(uint16_t slave_index, uint8_t module_index) const override;
};
}  // namespace march
#endif  // MARCH_HARDWARE_PDO_INTERFACE_H
