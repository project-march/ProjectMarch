//
// EtherCAT master class header. Interfaces with SOEM
//

#ifndef PROJECT_ETHERCAT_H
#define PROJECT_ETHERCAT_H

#include <march_hardware/Joint.h>

extern "C" {
#include "osal.h"
}

class EthercatMaster
{
  std::string ifname;  // Network interface name, check ifconfig
  char IOmap[4096];    // Holds the mapping of the SOEM message
  int expectedWKC;     // Expected working counter
  std::vector<Joint> jointList;    // Contains all slaves

public:
  bool inOP;  // Is SOEM in operational state

  // Constructor
  explicit EthercatMaster(std::vector<Joint> slaves);
  // Destructor
  ~EthercatMaster();
  // Send Process Data over EtherCAT
  void SendProcessData();
  // Receive Process Data over EtherCAT
  int ReceiveProcessData();
  // Publish received Process Data per slave
  void PublishProcessData();
  // Monitor slave connection
  void MonitorSlaveConnection();
  void EthercatLoop();

};

#endif  // PROJECT_ETHERCAT_H
