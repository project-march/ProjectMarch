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
 * @param io_map Holds the mapping of the SOEM message.
 * @param expeted_working_counter The expected working counter of the ethercat train.
 * @param cycle_time_ms The ethercat cycle time.
 * @param max_slave_index The maximum amount of slaves connected to the train.
 */
class EthercatMaster
{
public:
  EthercatMaster(std::vector<Joint>* joints_ptr, std::string ifname, int max_slave_index, int cycle_time);
  ~EthercatMaster();

  bool isOperational() const;

  void start();
  void stop();

private:
  void ethercatMasterInitiation();
  void ethercatSlaveInitiation();

  void ethercatLoop();
  void SendReceivePDO();
  static void monitorSlaveConnection();

  bool is_operational_ = false;

  std::vector<Joint>* joints_ptr_;

  const std::string ifname_;
  const int max_slave_index_;
  const int cycle_time_ms_;

  char io_map_[4096] = { 0 };
  int expected_working_counter_ = 0;

  std::thread ethercat_thread_;
};

}  // namespace march
#endif  // MARCH_HARDWARE_ETHERCAT_ETHERCATMASTER_H
