#ifndef MARCH_HARDWARE_SDO_INTERFACE_H
#define MARCH_HARDWARE_SDO_INTERFACE_H
#include <cstdint>

namespace march
{
/**
 * An interface to read and write Service Data Objects (SDOs).
 */
class SdoInterface
{
public:
  /**
   * Writes an SDO to given location.
   *
   * @tparam T type of the SDO to write
   * @param slave index of the slave
   * @param index index of the register
   * @param sub sub index inside the register
   * @param value value to write
   * @return received working counter from the read operation. Returns 0 when a failure occurred, positive otherwise.
   */
  template <typename T>
  int write(uint16_t slave, uint16_t index, uint8_t sub, T value)
  {
    return this->write(slave, index, sub, sizeof(T), &value);
  }

  /**
   * Reads an SDO from given location.
   *
   * @tparam T type to read SDO to
   * @param slave index of the slave
   * @param index index of the register
   * @param sub sub index inside the register
   * @param val_size reference to the size of the value. Will output the size of the read value.
   * @param value reference to value to read to
   * @return received working counter from the read operation. Returns 0 when a failure occurred, positive otherwise.
   */
  template <typename T>
  int read(uint16_t slave, uint16_t index, uint8_t sub, int& val_size, T& value)
  {
    return this->read(slave, index, sub, val_size, &value);
  }

protected:
  virtual int write(uint16_t slave, uint16_t index, uint8_t sub, std::size_t size, void* value) = 0;

  virtual int read(uint16_t slave, uint16_t index, uint8_t sub, int& val_size, void* value) = 0;
};

class SdoInterfaceImpl : public SdoInterface
{
protected:
  int write(uint16_t slave, uint16_t index, uint8_t sub, std::size_t size, void* value) override;

  int read(uint16_t slave, uint16_t index, uint8_t sub, int& val_size, void* value) override;
};
}
#endif  // MARCH_HARDWARE_SDO_INTERFACE_H
