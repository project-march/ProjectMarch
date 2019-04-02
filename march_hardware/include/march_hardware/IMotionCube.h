// Copyright 2019 Project March.
#ifndef MARCH4CPP__IMOTIONCUBE_H
#define MARCH4CPP__IMOTIONCUBE_H

#include <map>
#include <string>

#include <march_hardware/EtherCAT/EthercatIO.h>
#include <march_hardware/Slave.h>
#include <march_hardware/Encoder.h>
#include <march_hardware/PDOmap.h>

namespace march4cpp
{
class IMotionCube : public Slave
{
private:
  Encoder encoder;
  void actuateIU(int iu);
  bool get_bit(uint16 value, int index);
  std::map<IMCObjectName, int> misoByteOffsets;
  std::map<IMCObjectName, int> mosiByteOffsets;

public:
  explicit IMotionCube(int slaveIndex, Encoder encoder);

  IMotionCube()
  {
    slaveIndex = -1;
  }

  ~IMotionCube() = default;

  void writeInitialSDOs(int ecatCycleTime) override;

  void mapPDOs();

  bool writeInitialSettings(uint8 ecatCycleTime);

  float getAngleRad();

  uint16 getStatusWord();
  uint16 getMotionError();
  uint16 getDetailedError();

  void setControlWord(uint16 controlWord);

  void actuateRad(float targetRad);
  void actuateRadFixedSpeed(float targetRad, float radPerSec);

  void parseStatusWord(uint16 statusWord);
  void parseMotionError(uint16 motionError);
  void parseDetailedError(uint16 detailedError);

  bool goToOperationEnabled();
};

}  // namespace march4cpp
#endif  // MARCH4CPP__IMOTIONCUBE_H
