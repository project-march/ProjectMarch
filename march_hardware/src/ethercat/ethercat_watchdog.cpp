// Copyright 2019 Project March.
#include "march_hardware/ethercat/ethercat_watchdog.h"

#include <utility>

namespace march
{
EthercatWatchdog::EthercatWatchdog(const ros::Duration& timeout, std::function<void()> callback)
{
  this->callback_ = std::move(callback);
  this->timeout_ = timeout;
}

EthercatWatchdog::~EthercatWatchdog()
{
  this->active_ = false;
  this->loop_thread_.join();
}

bool EthercatWatchdog::isActive() const
{
  return this->active_;
}

void EthercatWatchdog::start()
{
  if (!this->active_)
  {
    this->active_ = true;
    this->last_pet_time_ = ros::Time::now();
    this->loop_thread_ = std::thread(&EthercatWatchdog::loop, this);
  }
}

void EthercatWatchdog::stop()
{
  if (this->active_)
  {
    this->active_ = false;
    this->loop_thread_.join();
  }
}

void EthercatWatchdog::pet()
{
  this->last_pet_time_ = ros::Time::now();
}

void EthercatWatchdog::loop()
{
  while (this->active_)
  {
    if ((ros::Time::now() - this->last_pet_time_) > this->timeout_)
    {
      break;
    }
  }

  if (this->active_)
  {
    this->active_ = false;
    this->callback_();
  }
}
}  // namespace march
