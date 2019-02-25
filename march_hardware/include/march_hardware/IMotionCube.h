//
// Created by projectmarch on 13-2-19.
//
#ifndef MARCH4CPP__IMOTIONCUBE_H
#define MARCH4CPP__IMOTIONCUBE_H

#include <march_hardware/EtherCAT/EthercatIO.h>
#include <march_hardware/Slave.h>
#include <march_hardware/Encoder.h>
#include "TemperatureSensor.h"

namespace march4cpp
{
class IMotionCube : public Slave
{
private:
  AngleEncoder* encoder;
  // TODO(Martijn) add PDO/SDO settings here.

public:
  explicit IMotionCube(int slaveIndex, AngleEncoder* encoder);

  IMotionCube()
  {
    slaveIndex = -1;
  }

  ~IMotionCube() = default;

  void initialize() override;

  bool PDOmapping();

  bool StartupSDO(uint8 ecatCycleTime);

  float getAngle();
};

}  // namespace march4cpp
#endif  // MARCH4CPP__IMOTIONCUBE_H
