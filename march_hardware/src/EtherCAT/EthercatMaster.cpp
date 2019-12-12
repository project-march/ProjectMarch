// Copyright 2019 Project March.

#include <string>
#include <vector>

#include <boost/chrono/chrono.hpp>

#include <ros/ros.h>

#include <march_hardware/EtherCAT/EthercatMaster.h>

#include <soem/ethercattype.h>
#include <soem/nicdrv.h>
#include <soem/ethercatbase.h>
#include <soem/ethercatmain.h>
#include <soem/ethercatdc.h>
#include <soem/ethercatcoe.h>
#include <soem/ethercatfoe.h>
#include <soem/ethercatconfig.h>
#include <soem/ethercatprint.h>

namespace march4cpp
{
EthercatMaster::EthercatMaster(std::vector<Joint>* jointListPtr, std::string ifname, int maxSlaveIndex,
                               int ecatCycleTime)
  : jointListPtr(jointListPtr)
{
  this->ifname = ifname;
  this->maxSlaveIndex = maxSlaveIndex;
  this->ecatCycleTimems = ecatCycleTime;
}

EthercatMaster::~EthercatMaster()
{
  std::cout << "Ethercat master has been deconstructed\n";
  this->stop();
}

/**
 * Initiate the ethercat train and start the loop
 */
void EthercatMaster::start()
{
  EthercatMaster::ethercatMasterInitiation();
  EthercatMaster::ethercatSlaveInitiation();
}

/**
 * Open the ethernet port with the given ifname and check amount of slaves
 */
void EthercatMaster::ethercatMasterInitiation()
{
  ROS_INFO("Trying to start EtherCAT");
  if (!ec_init(&ifname[0]))
  {
    ROS_FATAL("No socket connection on %s. Confirm that you have selected the right ifname", ifname.c_str());
    throw std::runtime_error("No socket connection on %s. Confirm that you have selected the right ifname");
  }
  ROS_INFO("ec_init on %s succeeded", ifname.c_str());

  if (ec_config_init(FALSE) <= 0)
  {
    ROS_FATAL("No slaves found, shutting down. Confirm that you have selected the right ifname.");
    ROS_FATAL("Check that the first slave is connected properly");
    throw std::runtime_error("No slaves found, shutting down. Confirm that you have selected the right ifname.");
  }
  ROS_INFO("%d slave(s) found and initialized.", ec_slavecount);

  if (ec_slavecount < this->maxSlaveIndex)
  {
    ROS_FATAL("Slave configured with index %d while soem only found %d slave(s)", this->maxSlaveIndex, ec_slavecount);
    throw std::runtime_error("More slaves configured than soem could detect.");
  }
}

/**
 * Set the found slaves to pre-operational state, configure the slaves and move to safe-operational state. If everything
 * went good move to operational state.
 */
void EthercatMaster::ethercatSlaveInitiation()
{
  ROS_INFO("Request per-operational state for all slaves");
  ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 4);

  for (auto& i : *jointListPtr)
  {
    i.initialize(ecatCycleTimems);
  }

  ec_config_map(&IOmap);
  ec_configdc();

  ROS_INFO("Request safe-operational state for all slaves");
  ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

  expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
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
    isOperational = true;
    EcatThread = std::thread(&EthercatMaster::ethercatLoop, this);
  }
  else
  {
    ROS_FATAL("Not all slaves reached operational state. Non-operational slave(s) listed below.");
    ec_readstate();
    for (int i = 1; i <= ec_slavecount; i++)
    {
      if (ec_slave[i].state != EC_STATE_OPERATIONAL)
      {
        ROS_INFO("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n", i, ec_slave[i].state, ec_slave[i].ALstatuscode,
                 ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
      }
    }
    throw std::runtime_error("Not all slaves reached operational state.");
  }
}

/**
 * The ethercat train PDO loop. If the working counter is lower then expected 5% of the time the program displays an
 * error
 */
void EthercatMaster::ethercatLoop()
{
  uint32_t totalLoops = 0;
  uint32_t rateNotAchievedCount = 0;
  int rate = 1000 / ecatCycleTimems;

  while (isOperational)
  {
    auto start = boost::chrono::high_resolution_clock::now();

    SendReceivePDO();
    monitorSlaveConnection();

    auto stop = boost::chrono::high_resolution_clock::now();
    auto duration = boost::chrono::duration_cast<boost::chrono::microseconds>(stop - start);

    if (duration.count() > ecatCycleTimems * 1000)
    {
      rateNotAchievedCount++;
    }
    else
    {
      usleep(ecatCycleTimems * 1000 - duration.count());
    }
    totalLoops++;

    if (totalLoops >= (10 * rate))
    {
      float rateNotAchievedPercentage = 100 * (static_cast<float>(rateNotAchievedCount) / totalLoops);
      if (rateNotAchievedPercentage > 5)
      {
        ROS_WARN("EtherCAT rate of %d milliseconds per cycle was not achieved for %f percent of all cycles",
                 ecatCycleTimems, rateNotAchievedPercentage);
      }
      else
      {
        ROS_DEBUG("EtherCAT rate of %d milliseconds per cycle was not achieved for %f percent of all cycles",
                  ecatCycleTimems, rateNotAchievedPercentage);
      }
      totalLoops = 0;
      rateNotAchievedCount = 0;
    }
  }
}

/**
 * Send the PDO and receive the working counter and check if this is lower then expected.
 */
void EthercatMaster::SendReceivePDO()
{
  ec_send_processdata();
  int wkc = ec_receive_processdata(EC_TIMEOUTRET);
  if (wkc < this->expectedWKC)
  {
    ROS_WARN_THROTTLE(1, "Working counter lower than expected. EtherCAT connection may not be optimal");
  }
}

/**
 * Check if all the slaves are connected and in operational state.
 */
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

/**
 * Stop the ethercat loop and terminate the thread
 */
void EthercatMaster::stop()
{
  if (this->isOperational)
  {
    ROS_INFO("Stopping EtherCAT");
    isOperational = false;
    EcatThread.join();

    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);
    ec_close();
  }
}

}  // namespace march4cpp