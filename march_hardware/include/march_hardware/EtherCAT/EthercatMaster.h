#ifndef MARCH4CPP__ETHERCATMASTER_H
#define MARCH4CPP__ETHERCATMASTER_H

#include <thread>

#include <march_hardware/Joint.h>

class EthercatMaster
{
  std::string ifname;  // Network interface name, check ifconfig
  char IOmap[4096];    // Holds the mapping of the SOEM message
  int expectedWKC;     // Expected working counter
  std::thread EcatThread; // Handler for parallel thread

  std::vector<Joint> jointList;

public:
  bool isOperational = false;  // Is SOEM in operational state

  explicit EthercatMaster(std::vector<Joint> slaves);

  void start();
  void stop();

  // Parallel thread
  void ethercatLoop();

  void sendProcessData();

  int receiveProcessData();

  void monitorSlaveConnection();
};

#endif  // PROJECT_ETHERCAT_H
