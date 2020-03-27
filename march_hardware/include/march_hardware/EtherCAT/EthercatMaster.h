// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_ETHERCAT_ETHERCATMASTER_H
#define MARCH_HARDWARE_ETHERCAT_ETHERCATMASTER_H
#include <atomic>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>

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

  /* Delete move constructor/assignment since atomic bool cannot be moved */
  EthercatMaster(EthercatMaster&&) = delete;
  EthercatMaster& operator=(EthercatMaster&&) = delete;

  bool isOperational() const;
  void waitForPdo();

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

  static const int THREAD_PRIORITY = 40;

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

  /**
   * Sets the ethercat thread priority and scheduling
   * to SCHED_FIFO using pthread.
   * Note: Only works on POSIX compliant systems.
   *
   * @param priority a pthread priority value between 1 and 99 for SCHED_FIFO threads.
   */
  void setThreadPriority(int priority);

  std::atomic<bool> is_operational_;

  const std::string ifname_;
  const int max_slave_index_;
  const int cycle_time_ms_;

  std::mutex wait_on_pdo_condition_mutex_;
  std::condition_variable wait_on_pdo_condition_var_;
  bool pdo_received_ = false;

  char io_map_[4096] = { 0 };
  int expected_working_counter_ = 0;

  std::thread ethercat_thread_;
};

}  // namespace march
#endif  // MARCH_HARDWARE_ETHERCAT_ETHERCATMASTER_H
