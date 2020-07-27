// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_ETHERCAT_WATCHDOG_H
#define MARCH_HARDWARE_ETHERCAT_WATCHDOG_H

#include <boost/function.hpp>
#include <mutex>
#include <thread>
#include <ros/ros.h>

namespace march
{
/**
 * Base class of a custom watchdog to monitor the ethercat slave connection
 */
class EthercatWatchdog
{
public:
  EthercatWatchdog(const ros::Duration& timeout, std::function<void()> callback);
  ~EthercatWatchdog();

  /**
   * Start the watchdog by initialising th loop thread.
   */
  void start();

  /**
   * Stop the watchdog thread.
   */
  void stop();

  /**
   * Update the watchdog timer.
   */
  void pet();

  /**
   * Check the current ros time compared to the latest watchdog update time.
   */
  void loop();

  /**
   * Return if the watchdog thread is active.
   */
  bool isActive() const;

private:
  ros::Duration timeout_;
  bool active_ = false;

  std::thread loop_thread_;
  ros::Time last_pet_time_;

  std::function<void()> callback_;
};

}  // namespace march
#endif  // MARCH_HARDWARE_ETHERCAT_WATCHDOG_H
