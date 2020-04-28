// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_IMOTIONCUBE_H
#define MARCH_HARDWARE_IMOTIONCUBE_H
#include "march_hardware/ActuationMode.h"
#include "march_hardware/EtherCAT/pdo_types.h"
#include "march_hardware/EtherCAT/sdo_interface.h"
#include "march_hardware/EtherCAT/slave.h"
#include "march_hardware/IMotionCubeState.h"
#include "march_hardware/IMotionCubeTargetState.h"
#include "march_hardware/PDOmap.h"
#include "march_hardware/encoder/AbsoluteEncoder.h"
#include "march_hardware/encoder/IncrementalEncoder.h"

#include <memory>
#include <unordered_map>
#include <string>

namespace march
{
class IMotionCube : public Slave
{
public:
  /**
   * Constructs an IMotionCube with an incremental and absolute encoder.
   *
   * @param slave_index index of the IMotionCube slave
   * @param absolute_encoder pointer to absolute encoder, required so cannot be nullptr
   * @param incremental_encoder pointer to incremental encoder, required so cannot be nullptr
   * @param actuation_mode actuation mode in which the IMotionCube must operate
   * @throws std::invalid_argument When an absolute or incremental encoder is nullptr.
   */
  IMotionCube(Slave slave, std::unique_ptr<AbsoluteEncoder> absolute_encoder,
              std::unique_ptr<IncrementalEncoder> incremental_encoder, ActuationMode actuation_mode);
  IMotionCube(Slave slave, std::unique_ptr<AbsoluteEncoder> absolute_encoder,
              std::unique_ptr<IncrementalEncoder> incremental_encoder, std::string& sw_stream,
              ActuationMode actuation_mode);

  ~IMotionCube() noexcept override = default;

  /* Delete copy constructor/assignment since the unique_ptrs cannot be copied */
  IMotionCube(const IMotionCube&) = delete;
  IMotionCube& operator=(const IMotionCube&) = delete;

  virtual double getAngleRadAbsolute();
  virtual double getAngleRadIncremental();
  double getAbsoluteRadPerBit() const;
  double getIncrementalRadPerBit() const;
  int16_t getTorque();
  int32_t getAngleIUAbsolute();
  int32_t getAngleIUIncremental();

  uint16_t getStatusWord();
  uint16_t getMotionError();
  uint16_t getDetailedError();

  ActuationMode getActuationMode() const;

  virtual float getMotorCurrent();
  virtual float getIMCVoltage();

  void setControlWord(uint16_t control_word);

  virtual void actuateRad(double target_rad);
  virtual void actuateTorque(int16_t target_torque);

  void goToTargetState(IMotionCubeTargetState target_state);
  virtual void goToOperationEnabled();

  /** @brief Override comparison operator */
  friend bool operator==(const IMotionCube& lhs, const IMotionCube& rhs)
  {
    return lhs.getSlaveIndex() == rhs.getSlaveIndex() && *lhs.absolute_encoder_ == *rhs.absolute_encoder_ &&
           *lhs.incremental_encoder_ == *rhs.incremental_encoder_;
  }
  /** @brief Override stream operator for clean printing */
  friend std::ostream& operator<<(std::ostream& os, const IMotionCube& imc)
  {
    return os << "slaveIndex: " << imc.getSlaveIndex() << ", "
              << "incrementalEncoder: " << *imc.incremental_encoder_ << ", "
              << "absoluteEncoder: " << *imc.absolute_encoder_;
  }

  constexpr static double MAX_TARGET_DIFFERENCE = 0.393;
  // This value is slightly larger than the current limit of the
  // linear joints defined in the URDF.
  const static int16_t MAX_TARGET_TORQUE = 23500;

  // Watchdog base time = 1 / 25 MHz * (2498 + 2) = 0.0001 seconds=100 µs
  static const uint16_t WATCHDOG_DIVIDER = 2498;
  // 500 * 100us = 50 ms = watchdog timer
  static const uint16_t WATCHDOG_TIME = 500;

protected:
  void initSdo(SdoInterface& sdo, int cycle_time) override;

  void reset(SdoInterface& sdo) override;

private:
  void actuateIU(int32_t target_iu);

  void mapMisoPDOs(SdoInterface& sdo);
  void mapMosiPDOs(SdoInterface& sdo);
  void writeInitialSettings(SdoInterface& sdo, int cycle_time);
  /**
   * Calculates checksum on .sw file passed in string format in sw_string_ by simple summation until next empty line.
   * Start_address and end_address are filled in the method and used for downloading the .sw file to the drive.
   */
  uint32_t computeSWCheckSum(int& start_address, int& end_address);

  // Use of smart pointers are necessary here to make dependency injection
  // possible and thus allow for mocking the encoders. A unique pointer is
  // chosen since the IMotionCube should be the owner and the encoders
  // do not need to be passed around.
  std::unique_ptr<AbsoluteEncoder> absolute_encoder_;
  std::unique_ptr<IncrementalEncoder> incremental_encoder_;
  std::string sw_string_;
  ActuationMode actuation_mode_;

  std::unordered_map<IMCObjectName, uint8_t> miso_byte_offsets_;
  std::unordered_map<IMCObjectName, uint8_t> mosi_byte_offsets_;
};

}  // namespace march
#endif  // MARCH_HARDWARE_IMOTIONCUBE_H
