// Copyright 2019 Project March.

//
// EtherCAT master class source. Interfaces with SOEM
//

#include <boost/chrono/chrono.hpp>

#include <ros/ros.h>

#include <march_hardware/EtherCAT/EthercatMaster.h>

extern "C"
{
#include "ethercat.h"
}

namespace march4cpp
{
// Constructor
EthercatMaster::EthercatMaster(std::vector<Joint> *jointListPtr, std::string ifname, int maxSlaveIndex,
                               int ecatCycleTime) : jointListPtr(jointListPtr)
{
  this->ifname = ifname;
  this->maxSlaveIndex = maxSlaveIndex;
  this->ecatCycleTimems = ecatCycleTime;
}

void EthercatMaster::start()
{
  // TODO(Isha, Martijn) this method is really long, split into more methods
  ROS_INFO("Trying to start EtherCAT");

  // Initialise SOEM, bind socket to ifname
  if (!ec_init(ifname.c_str()))
  {
    ROS_ERROR("No socket connection on %s. Confirm that you have selected the right ifname", ifname.c_str());
    return;
  }
  ROS_INFO("ec_init on %s succeeded", ifname.c_str());

  // Find and auto-config slaves
  if (ec_config_init(FALSE) <= 0)
  {
    ROS_ERROR("No slaves found, shutting down. Confirm that you have selected the right ifname.");
    ROS_ERROR("Check that the first slave is connected properly");
    return;
  }
  ROS_INFO("%d slave(s) found and initialized.", ec_slavecount);

  if (ec_slavecount < this->maxSlaveIndex)
  {
    ROS_FATAL("Slave configured with index %d while soem only found %d slave(s)", this->maxSlaveIndex, ec_slavecount);
    return;
  }
  // TODO(Martijn) Check on type of slaves

  // Request and wait for slaves to be in preOP state
  ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 4);

  for (int i = 0; i < jointListPtr->size(); i++)
  {
    jointListPtr->at(i).initialize(ecatCycleTimems);
  }

  // Configure the EtherCAT message structure depending on the PDO mapping of all the slaves
  ec_config_map(&IOmap);

  ec_configdc();

  // Wait for all slaves to reach SAFE_OP state
  ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

  ROS_INFO("Request operational state for all slaves");
  expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
  ec_slave[0].state = EC_STATE_OPERATIONAL;

  // send one valid process data to make outputs in slaves happy
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);

  // request OP state for all slaves
  ec_writestate(0);
  int chk = 40;

  /* wait for all slaves to reach OP state */
  do
  {
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
  }
  while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

  if (ec_slave[0].state == EC_STATE_OPERATIONAL)
  {
    // All slaves in operational state
    ROS_INFO("Operational state reached for all slaves");
    isOperational = true;
    EcatThread = std::thread(&EthercatMaster::ethercatLoop, this);
  }
  else
  {
    // Not all slaves in operational state
    ROS_FATAL("Not all slaves reached operational state. See below for more information on which slave.");
    ec_readstate();
    for (int i = 1; i <= ec_slavecount; i++)
    {
      if (ec_slave[i].state != EC_STATE_OPERATIONAL)
      {
        ROS_INFO("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n", i, ec_slave[i].state, ec_slave[i].ALstatuscode,
                 ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
      }
    }
    throw std::runtime_error("Not all slaves reached operational state.")
  }
}

void EthercatMaster::stop()
{
  ROS_INFO("Stopping EtherCAT");
  isOperational = false;
  EcatThread.join();
  ec_slave[0].state = EC_STATE_INIT;
  ec_writestate(0);
  ec_close();
}

void EthercatMaster::ethercatLoop()
{
  while (isOperational)
  {
    auto start = boost::chrono::high_resolution_clock::now();
    sendProcessData();
    receiveProcessData();
    monitorSlaveConnection();
    auto stop = boost::chrono::high_resolution_clock::now();
    auto duration = boost::chrono::duration_cast<boost::chrono::microseconds>(stop - start);
    if (duration.count() > ecatCycleTimems * 1000)
    {
        ROS_WARN("EtherCAT rate of %d milliseconds per cycle was not achieved this EtherCAT cycle", ecatCycleTimems);
    }
    else
    {
        usleep(ecatCycleTimems * 1000 - duration.count());
    }
  }
}

void EthercatMaster::sendProcessData()
{
  ec_send_processdata();
}

int EthercatMaster::receiveProcessData()
{
  return ec_receive_processdata(EC_TIMEOUTRET);
}

void EthercatMaster::monitorSlaveConnection()
{
  for (int slave = 1; slave <= ec_slavecount; slave++)
  {
    ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
    if (ec_slave[slave].state == EC_STATE_NONE)
    {
      ROS_ERROR("EtherCAT train lost connection from slave %d onwards", slave);
      throw std::exception();
    }
  }
}

}  // namespace march4cpp
