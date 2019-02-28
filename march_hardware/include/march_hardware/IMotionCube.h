//
// Created by projectmarch on 13-2-19.
//
#ifndef MARCH4CPP__IMOTIONCUBE_H
#define MARCH4CPP__IMOTIONCUBE_H

#include <march_hardware/EtherCAT/EthercatIO.h>
#include <march_hardware/Slave.h>
#include <march_hardware/Encoder.h>

namespace march4cpp
{
class IMotionCube : public Slave
{
private:
  Encoder encoder;
  void actuateIU(int iu);
  // TODO(Martijn) add PDO/SDO settings here.

public:
  explicit IMotionCube(int slaveIndex, Encoder encoder);

  IMotionCube()
  {
    slaveIndex = -1;
  }

  ~IMotionCube() = default;

  void initialize() override;

  bool PDOmapping();

  bool StartupSDO(uint8 ecatCycleTime);

  float getAngleRad();

  uint16 getMotionError();
  uint16 getDetailedError();
  uint16 getStatusWord();

  void setControlWord(uint16 controlWord);

  void actuateRad(float targetRad);

  void parseMotionError(int errorCode);
  void parseStatusWord();
  bool goToOperationEnabled();

  bool get_bit(uint16 value, int index);
};

}  // namespace march4cpp
#endif  // MARCH4CPP__IMOTIONCUBE_H
