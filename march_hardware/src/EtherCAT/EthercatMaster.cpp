// Copyright 2019 Project March.
#include "march_hardware/EtherCAT/EthercatMaster.h"
#include "march_hardware/error/hardware_exception.h"

#include <sstream>
#include <string>
#include <vector>

#include <boost/chrono/chrono.hpp>

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
  : ifname_(std::move(ifname)), max_slave_index_(max_slave_index), cycle_time_ms_(cycle_time)
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

void EthercatMaster::start(std::vector<Joint>& joints)
{
  EthercatMaster::ethercatMasterInitiation();
  EthercatMaster::ethercatSlaveInitiation(joints);
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
    joint.initialize(cycle_time_ms_);
  }

  ec_config_map(&io_map_);
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
    ethercat_thread_ = std::thread(&EthercatMaster::ethercatLoop, this);
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
  size_t totalLoops = 0;
  size_t rateNotAchievedCount = 0;
  size_t rate = 1000 / cycle_time_ms_;

  while (this->is_operational_)
  {
    auto start = boost::chrono::high_resolution_clock::now();

    SendReceivePDO();
    monitorSlaveConnection();

    auto stop = boost::chrono::high_resolution_clock::now();
    auto duration = boost::chrono::duration_cast<boost::chrono::microseconds>(stop - start);

    if (duration.count() > cycle_time_ms_ * 1000)
    {
      rateNotAchievedCount++;
    }
    else
    {
      usleep(cycle_time_ms_ * 1000 - duration.count());
    }
    totalLoops++;

    if (totalLoops >= (10 * rate))
    {
      float rateNotAchievedPercentage = 100 * (static_cast<float>(rateNotAchievedCount) / totalLoops);
      if (rateNotAchievedPercentage > 5)
      {
        ROS_WARN("EtherCAT rate of %d milliseconds per cycle was not achieved for %f percent of all cycles",
                 cycle_time_ms_, rateNotAchievedPercentage);
      }
      else
      {
        ROS_DEBUG("EtherCAT rate of %d milliseconds per cycle was not achieved for %f percent of all cycles",
                  cycle_time_ms_, rateNotAchievedPercentage);
      }
      totalLoops = 0;
      rateNotAchievedCount = 0;
    }
  }
}

void EthercatMaster::SendReceivePDO()
{
  ec_send_processdata();
  int wkc = ec_receive_processdata(EC_TIMEOUTRET);
  if (wkc < this->expected_working_counter_)
  {
    ROS_WARN_THROTTLE(1, "Working counter lower than expected. EtherCAT connection may not be optimal");
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

}  // namespace march
