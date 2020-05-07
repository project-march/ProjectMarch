// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_PDOMAP_H
#define MARCH_HARDWARE_PDOMAP_H

#include <string>
#include <utility>
#include <vector>
#include <unordered_map>

#include <ros/ros.h>

#include <march_hardware/EtherCAT/EthercatSDO.h>

namespace march
{
/** Store IMC data as a struct to prevent data overlap.*/
struct IMCObject
{
  uint16_t address;           // in IMC memory (see IMC manual)
  uint16_t length;            // bits (see IMC manual)
  uint32_t combined_address;  // combine the address(hex), sub-index(hex) and length(hex)

  explicit IMCObject(uint16_t _address, uint16_t _length) : address(_address), length(_length)
  {
    uint32_t MSword = ((address & 0xFFFF) << 16);  // Shift 16 bits left for most significant word
    uint32_t LSword = (length & 0xFFFF);

    combined_address = (MSword | LSword);
  }

  IMCObject(){};
};

/** The data direction to which the PDO is specified is restricted to master in slave out and slave out master in.*/
enum class DataDirection
{
  MISO,
  MOSI,
};

/** All the available IMC object names divided over the PDO maps. make sure to also add it to PDOmap constructor.*/
enum class IMCObjectName
{
  StatusWord,
  ActualPosition,
  ActualVelocity,
  MotionErrorRegister,
  DetailedErrorRegister,
  DCLinkVoltage,
  DriveTemperature,
  ActualTorque,
  CurrentLimit,
  MotorPosition,
  MotorVelocity,
  ControlWord,
  TargetPosition,
  TargetTorque,
  QuickStopDeceleration,
  QuickStopOption
};

class PDOmap
{
public:
  /**
   * Initiate all the entered IMC objects to prepare the PDO.
   *
   * @param object_name enum of the object to be added.
   * @throws HardwareException when the object to be added is not defined or the registers overflow.
   */
  void addObject(IMCObjectName object_name);

  std::unordered_map<IMCObjectName, uint8_t> map(int slave_index, DataDirection direction);

  static std::unordered_map<IMCObjectName, IMCObject> all_objects;

private:
  /** Used to sort the objects in the all_objects according to data length.
   * @return list of pairs <IMCObjectName, IMCObjects> according from object sizes */
  std::vector<std::pair<IMCObjectName, IMCObject>> sortPDOObjects();

  /** Configures the PDO in the IMC using the given base register address and sync manager address.
   * @return map of the IMC PDO object name in combination with the byte-offset in the PDO register */
  std::unordered_map<IMCObjectName, uint8_t> configurePDO(int slave_index, int base_register,
                                                          uint16_t base_sync_manager);

  std::unordered_map<IMCObjectName, IMCObject> PDO_objects;
  int total_used_bits = 0;

  const int bits_per_register = 64;           // Maximum amount of bits that can be constructed in one PDO message.
  const int nr_of_regs = 4;                   // Amount of registers available.
  const int object_sizes[3] = { 32, 16, 8 };  // Available sizes.
};
}  // namespace march

#endif  // MARCH_HARDWARE_PDOMAP_H
