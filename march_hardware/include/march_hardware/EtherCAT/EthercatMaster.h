// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_ETHERCAT_ETHERCATMASTER_H
#define MARCH_HARDWARE_ETHERCAT_ETHERCATMASTER_H
#include <vector>
#include <string>
#include <thread>

#include <march_hardware/Joint.h>

namespace march
{
/**
 * Base class of the ethercat master supported with the SOEM library
 * @param ifname Network interface name, check ifconfig.
 * @param IOmap Holds the mapping of the SOEM message.
 * @param expectedWKC The expected working counter of the ethercat train.
 * @param ecatCycleTimems The ethercat cycle time.
 * @param maxSlaveIndex The maximum amount of slaves connected to the train.
 */
class EthercatMaster
{
  std::string ifname;
  char IOmap[4096];
  int expectedWKC;

  std::thread EcatThread;

  int maxSlaveIndex;
  int ecatCycleTimems;

public:
  bool isOperational = false;

  explicit EthercatMaster(std::string ifname, int maxSlaveIndex, int ecatCycleTime);
  ~EthercatMaster();

  /* Delete copy constructor/assignment since the member thread can not be copied */
  EthercatMaster(const EthercatMaster&) = delete;
  EthercatMaster& operator=(const EthercatMaster&) = delete;

  /* Enable the move constructor and assignment */
  EthercatMaster(EthercatMaster&&) = default;
  EthercatMaster& operator=(EthercatMaster&&) = default;

  /**
   * Initializes the ethercat train and starts a thread for the loop.
   * @throws HardwareException If not the configured amount of slaves was found
   *                           or they did not all reach operational state
   */
  void start(std::vector<Joint>& joints);

  /**
   * Stops the ethercat loop and joins the thread.
   */
  void stop();

private:
  /**
   * Opens the ethernet port with the given ifname and checks the amount of slaves.
   */
  void ethercatMasterInitiation();

  /**
   * Configures the found slaves to operational state.
   */
  void ethercatSlaveInitiation(std::vector<Joint>& joints);

  /**
   * The ethercat train PDO loop. If the working counter is lower than
   * expected 5% of the time, the program displays an error.
   */
  void ethercatLoop();

  /**
   * Sends the PDO and receives the working counter and check if this is lower than expected.
   */
  void SendReceivePDO();

  /**
   * Checks if all the slaves are connected and in operational state.
   */
  static void monitorSlaveConnection();
};

}  // namespace march
#endif  // MARCH_HARDWARE_ETHERCAT_ETHERCATMASTER_H
