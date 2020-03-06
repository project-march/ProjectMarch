// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_ETHERCAT_ETHERCATMASTER_H
#define MARCH_HARDWARE_ETHERCAT_ETHERCATMASTER_H
#include <atomic>
#include <vector>
#include <string>
#include <thread>

#include <march_hardware/Joint.h>

namespace march
{
/**
 * Base class of the ethercat master supported with the SOEM library
 * @param ifname Network interface name, check ifconfig.
 * @param io_map Holds the mapping of the SOEM message.
 * @param expected_working_counter The expected working counter of the ethercat train.
 * @param cycle_time_ms The ethercat cycle time.
 * @param max_slave_index The maximum amount of slaves connected to the train.
 */
class EthercatMaster
{
public:
  EthercatMaster(std::string ifname, int max_slave_index, int cycle_time);
  ~EthercatMaster();

  /* Delete copy constructor/assignment since the member thread can not be copied */
  EthercatMaster(const EthercatMaster&) = delete;
  EthercatMaster& operator=(const EthercatMaster&) = delete;

  /* Delete move constructor/assignment since atomic bool can not be moved */
  EthercatMaster(EthercatMaster&&) = delete;
  EthercatMaster& operator=(EthercatMaster&&) = delete;

  bool isOperational() const;

  /**
   * Returns the cycle time in milliseconds.
   */
  int getCycleTime() const;

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
  void sendReceivePdo();

  /**
   * Checks if all the slaves are connected and in operational state.
   */
  static void monitorSlaveConnection();

  std::atomic<bool> is_operational_;

  const std::string ifname_;
  const int max_slave_index_;
  const int cycle_time_ms_;

  char io_map_[4096] = { 0 };
  int expected_working_counter_ = 0;

  std::thread ethercat_thread_;
};

}  // namespace march
#endif  // MARCH_HARDWARE_ETHERCAT_ETHERCATMASTER_H
