// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_IMOTIONCUBE_H
#define MARCH_HARDWARE_IMOTIONCUBE_H

#include <map>
#include <string>

#include <march_hardware/ActuationMode.h>
#include <march_hardware/EtherCAT/EthercatIO.h>
#include <march_hardware/Slave.h>
#include <march_hardware/Encoder.h>
#include <march_hardware/PDOmap.h>
#include <march_hardware/IMotionCubeState.h>
#include <march_hardware/IMotionCubeTargetState.h>

namespace march
{
class IMotionCube : public Slave
{
public:
  IMotionCube(int slave_index, Encoder encoder, ActuationMode actuation_mode);

  ~IMotionCube() = default;

  void writeInitialSDOs(int ecatCycleTime) override;

  float getAngleRad();
  float getTorque();
  int getAngleIU();

  uint16 getStatusWord();
  uint16 getMotionError();
  uint16 getDetailedError();

  ActuationMode getActuationMode() const;

  float getMotorCurrent();
  float getMotorVoltage();

  void setControlWord(uint16 controlWord);

  void actuateRad(float targetRad);
  void actuateTorque(int targetTorque);

  std::string parseStatusWord(uint16 statusWord);
  IMCState getState(uint16 statusWord);
  std::string parseMotionError(uint16 motionError);
  std::string parseDetailedError(uint16 detailedError);

  bool goToTargetState(march::IMotionCubeTargetState targetState);
  bool goToOperationEnabled();
  bool resetIMotionCube();

  /** @brief Override comparison operator */
  friend bool operator==(const IMotionCube& lhs, const IMotionCube& rhs)
  {
    return lhs.slaveIndex == rhs.slaveIndex && lhs.encoder == rhs.encoder;
  }
  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const IMotionCube& iMotionCube)
  {
    return os << "slaveIndex: " << iMotionCube.slaveIndex << ", "
              << "encoder: " << iMotionCube.encoder;
  }

private:
  void actuateIU(int iu);

  void mapMisoPDOs();
  void mapMosiPDOs();
  void validateMisoPDOs();
  void validateMosiPDOs();
  void writeInitialSettings(uint8 ecatCycleTime);

  bool get_bit(uint16 value, int index);

  Encoder encoder;
  ActuationMode actuationMode;

  std::map<IMCObjectName, int> misoByteOffsets;
  std::map<IMCObjectName, int> mosiByteOffsets;
};

}  // namespace march
#endif  // MARCH_HARDWARE_IMOTIONCUBE_H
