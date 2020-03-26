// Copyright 2019 Project March.
#include "march_hardware/EtherCAT/EthercatMaster.h"
#include "march_hardware/error/hardware_exception.h"

#include <chrono>
#include <sstream>
#include <string>
#include <vector>

#include <pthread.h>
#include <ros/ros.h>

#include <soem/ethercattype.h>
#include <soem/nicdrv.h>
#include <soem/ethercatbase.h>
#include <soem/ethercatmain.h>
#include <soem/ethercatdc.h>
#include <soem/ethercatcoe.h>
#include <soem/ethercatfoe.h>
#include <soem/ethercatconfig.h>
#include <soem/ethercatprint.h>

namespace march
{
EthercatMaster::EthercatMaster(std::string ifname, int max_slave_index, int cycle_time)
  : is_operational_(false), ifname_(std::move(ifname)), max_slave_index_(max_slave_index), cycle_time_ms_(cycle_time)
{
}

EthercatMaster::~EthercatMaster()
{
  this->stop();
}

bool EthercatMaster::isOperational() const
{
  return this->is_operational_;
}

int EthercatMaster::getCycleTime() const
{
  return this->cycle_time_ms_;
}

void EthercatMaster::waitForPdo()
{
  std::unique_lock<std::mutex> lock(this->wait_on_pdo_condition_mutex_);
  this->wait_on_pdo_condition_var_.wait(lock, [&] { return this->pdo_received_; });
  this->pdo_received_ = false;
}

void EthercatMaster::start(std::vector<Joint>& joints)
{
  this->ethercatMasterInitiation();
  this->ethercatSlaveInitiation(joints);
}

void EthercatMaster::ethercatMasterInitiation()
{
  ROS_INFO("Trying to start EtherCAT");
  if (!ec_init(this->ifname_.c_str()))
  {
    throw error::HardwareException(error::ErrorType::NO_SOCKET_CONNECTION, "No socket connection on %s",
                                   this->ifname_.c_str());
  }
  ROS_INFO("ec_init on %s succeeded", this->ifname_.c_str());

  const int slave_count = ec_config_init(FALSE);
  if (slave_count < this->max_slave_index_)
  {
    ec_close();
    throw error::HardwareException(error::ErrorType::NOT_ALL_SLAVES_FOUND,
                                   "%d slaves configured while soem only found %d slave(s)", this->max_slave_index_,
                                   slave_count);
  }
  ROS_INFO("%d slave(s) found and initialized.", slave_count);
}

int setSlaveWatchdogTimer(uint16 slave)
{
  uint16 configadr = ec_slave[slave].configadr;
  ec_FPWRw(configadr, 0x0400, IMotionCube::WATCHDOG_DIVIDER, EC_TIMEOUTRET);  // Set the divider register of the WD
  ec_FPWRw(configadr, 0x0410, IMotionCube::WATCHDOG_TIME, EC_TIMEOUTRET);     // Set the PDI watchdog = WD
  ec_FPWRw(configadr, 0x0420, IMotionCube::WATCHDOG_TIME, EC_TIMEOUTRET);     // Set the SM watchdog = WD
  return 1;
}

void EthercatMaster::ethercatSlaveInitiation(std::vector<Joint>& joints)
{
  ROS_INFO("Request pre-operational state for all slaves");
  ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 4);

  for (Joint& joint : joints)
  {
    if (joint.hasIMotionCube())
    {
      ec_slave[joint.getIMotionCubeSlaveIndex()].PO2SOconfig = setSlaveWatchdogTimer;
    }
    joint.initialize(this->cycle_time_ms_);
  }

  ec_config_map(&this->io_map_);
  ec_configdc();

  ROS_INFO("Request safe-operational state for all slaves");
  ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

  this->expected_working_counter_ = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
  ec_slave[0].state = EC_STATE_OPERATIONAL;

  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);

  ROS_INFO("Request operational state for all slaves");
  ec_writestate(0);
  int chk = 40;

  do
  {
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
  } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

  if (ec_slave[0].state == EC_STATE_OPERATIONAL)
  {
    ROS_INFO("Operational state reached for all slaves");
    this->is_operational_ = true;
    this->ethercat_thread_ = std::thread(&EthercatMaster::ethercatLoop, this);
    this->setThreadPriority(EthercatMaster::THREAD_PRIORITY);
  }
  else
  {
    ec_readstate();
    std::ostringstream ss;
    for (int i = 1; i <= ec_slavecount; i++)
    {
      if (ec_slave[i].state != EC_STATE_OPERATIONAL)
      {
        ss << std::endl
           << "Slave " << i << " State=" << std::hex << std::showbase << ec_slave[i].state
           << " StatusCode=" << ec_slave[i].ALstatuscode << " (" << ec_ALstatuscode2string(ec_slave[i].ALstatuscode)
           << ")";
      }
    }
    throw error::HardwareException(error::ErrorType::FAILED_TO_REACH_OPERATIONAL_STATE, "Not operational slaves: %s",
                                   ss.str().c_str());
  }
}

void EthercatMaster::ethercatLoop()
{
  size_t total_loops = 0;
  size_t not_achieved_count = 0;
  const size_t rate = 1000 / this->cycle_time_ms_;
  const std::chrono::milliseconds cycle_time(this->cycle_time_ms_);

  while (this->is_operational_)
  {
    const auto begin_time = std::chrono::high_resolution_clock::now();

    this->sendReceivePdo();
    monitorSlaveConnection();

    const auto end_time = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - begin_time);

    {
      std::lock_guard<std::mutex> lock(this->wait_on_pdo_condition_mutex_);
      this->pdo_received_ = true;
    }
    this->wait_on_pdo_condition_var_.notify_one();

    if (duration > cycle_time)
    {
      not_achieved_count++;
    }
    else
    {
      std::this_thread::sleep_for(cycle_time - duration);
    }
    total_loops++;

    if (total_loops >= 10 * rate)
    {
      const double not_achieved_percentage = 100.0 * ((double)not_achieved_count / total_loops);
      if (not_achieved_percentage > 5.0)
      {
        ROS_WARN("EtherCAT rate of %d milliseconds per cycle was not achieved for %f percent of all cycles",
                 this->cycle_time_ms_, not_achieved_percentage);
      }
      total_loops = 0;
      not_achieved_count = 0;
    }
  }
}

void EthercatMaster::sendReceivePdo()
{
  ec_send_processdata();
  int wkc = ec_receive_processdata(EC_TIMEOUTRET);
  if (wkc < this->expected_working_counter_)
  {
    ROS_WARN_THROTTLE(1, "Working counter: %d  is lower than expected: %d", wkc, this->expected_working_counter_);
  }
}

void EthercatMaster::monitorSlaveConnection()
{
  for (int slave = 1; slave <= ec_slavecount; slave++)
  {
    ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
    if (!ec_slave[slave].state)
    {
      ROS_WARN_THROTTLE(1, "EtherCAT train lost connection from slave %d onwards", slave);
      return;
    }
  }
}

void EthercatMaster::stop()
{
  if (this->is_operational_)
  {
    ROS_INFO("Stopping EtherCAT");
    this->is_operational_ = false;
    this->ethercat_thread_.join();

    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);
    ec_close();
  }
}

void EthercatMaster::setThreadPriority(int priority)
{
  struct sched_param param = { priority };
  // SCHED_FIFO scheduling preempts other threads with lower priority as soon as it becomes runnable.
  // See http://man7.org/linux/man-pages/man7/sched.7.html for more info.
  const int error = pthread_setschedparam(this->ethercat_thread_.native_handle(), SCHED_FIFO, &param);
  if (error != 0)
  {
    ROS_ERROR("Failed to set the ethercat thread priority to %d. (error code: %d)", priority, error);
  }
  else
  {
    ROS_DEBUG("Set ethercat thread priority to %d", param.sched_priority);
  }
}
}  // namespace march
