// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_IMOTIONCUBE_H
#define MARCH_HARDWARE_IMOTIONCUBE_H
#include <march_hardware/ActuationMode.h>
#include <march_hardware/Encoder.h>
#include <march_hardware/EtherCAT/EthercatIO.h>
#include <march_hardware/IMotionCubeState.h>
#include <march_hardware/IMotionCubeTargetState.h>
#include <march_hardware/PDOmap.h>
#include <march_hardware/Slave.h>

#include <unordered_map>
#include <string>

namespace march
{
class IMotionCube : public Slave
{
public:
  IMotionCube(int slave_index, Encoder encoder, ActuationMode actuation_mode);

  ~IMotionCube() = default;

  void writeInitialSDOs(int cycle_time) override;

  float getAngleRad();
  float getTorque();
  int getAngleIU();

  uint16_t getStatusWord();
  uint16_t getMotionError();
  uint16_t getDetailedError();

  ActuationMode getActuationMode() const;

  float getMotorCurrent();
  float getMotorVoltage();

  void setControlWord(uint16_t control_word);

  void actuateRad(float target_rad);
  void actuateTorque(int target_torque);

  void goToTargetState(IMotionCubeTargetState target_state);
  void goToOperationEnabled();
  void resetIMotionCube();

  /** @brief Override comparison operator */
  friend bool operator==(const IMotionCube& lhs, const IMotionCube& rhs)
  {
    return lhs.slaveIndex == rhs.slaveIndex && lhs.encoder_ == rhs.encoder_;
  }
  /** @brief Override stream operator for clean printing */
  friend std::ostream& operator<<(std::ostream& os, const IMotionCube& imc)
  {
    return os << "slaveIndex: " << imc.slaveIndex << ", "
              << "encoder: " << imc.encoder_;
  }

private:
  void actuateIU(int target_iu);

  void mapMisoPDOs();
  void mapMosiPDOs();
  void writeInitialSettings(uint8_t cycle_time);

  Encoder encoder_;
  ActuationMode actuation_mode_;

  std::unordered_map<IMCObjectName, int> miso_byte_offsets_;
  std::unordered_map<IMCObjectName, int> mosi_byte_offsets_;
};

}  // namespace march
#endif  // MARCH_HARDWARE_IMOTIONCUBE_H
