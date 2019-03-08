#ifndef MARCH4CPP__ETHERCATMASTER_H
#define MARCH4CPP__ETHERCATMASTER_H

#include <thread>

#include <march_hardware/Joint.h>

namespace march4cpp
{
class EthercatMaster
{
  std::string ifname;      // Network interface name, check ifconfig
  char IOmap[4096];        // Holds the mapping of the SOEM message
  int expectedWKC;         // Expected working counter
  std::thread EcatThread;  // Handler for parallel thread
  // TODO (Isha, Tim) remove double joint list (also in March4)
  std::vector<Joint> jointList;
  int maxSlaveIndex;
  int ecatCycleTimems;

public:
  bool isOperational = false;  // Is SOEM in operational state

  explicit EthercatMaster(std::vector<Joint> jointList, std::string ifname, int maxSlaveIndex, int ecatCycleTime);

  void start();
  void stop();

  // Parallel thread
  void ethercatLoop();

  void sendProcessData();

  int receiveProcessData();

  void monitorSlaveConnection();
};

}  // namespace march4cpp
#endif
