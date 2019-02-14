#ifndef MARCH4CPP__MARCH4_H
#define MARCH4CPP__MARCH4_H

#include <string>
#include <vector>
#include <stdint.h>

#include <march_hardware/Joint.h>

#include <march_hardware/EtherCAT/EthercatMaster.h>

class MARCH4
{
private:
  EthercatMaster* ethercatMaster;

public:
  std::vector<Joint> jointList;

  MARCH4();

  void startEtherCAT();
  void stopEtherCAT();

  int getMaxSlaveIndex();

  bool hasValidSlaves();
  bool isOperational();
  Joint getJoint(std::string jointName);

  void sendData(uint8_t value);
};

#endif