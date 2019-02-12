//
// EtherCAT master class source. Interfaces with SOEM
//

#include <ros/ros.h>
#include "EthercatMaster.h"
#include "ethercat_SDO.h"
#include "ethercat_safety.h"
#include "Slaves/IMC.h"
#include "Slaves/TemplateGES.h"

extern "C" {
#include "ethercat.h"
}

// Constructor
EthercatMaster::EthercatMaster(ros::NodeHandle nh, std::vector<Slave*> slaves)
{
  slaveList = slaves;
  // Get the name of the network interface (if) over which SOEM will be run from the launch file
  nh.getParam(ros::this_node::getName() + "/ifname", ifname);

  inOP = false;

  printf("Starting ethercat\n");

  // Initialise SOEM, bind socket to ifname
  if (!ec_init(ifname.c_str()))
  {
    ROS_ERROR("No socket connection on %s", ifname.c_str());
    return;
  }
  printf("ec_init on %s succeeded.\n", ifname.c_str());

  // Find and auto-config slaves
  if (ec_config_init(FALSE) <= 0)
  {
    ROS_ERROR("No slaves found, shutting down");
    return;
  }
  printf("%d slaves found and configured.\n", ec_slavecount);

  // Request and wait for slaves to be in preOP state
  // Todo: Change to 0
  ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 4);

  // Over all iMotionCubes:
  //    Apply PDO-mapping while in PreOP state
  //    Send initialization SDO messages
  int ecatCycleTime;
  nh.getParam(ros::this_node::getName() + "/EthercatCycleTime", ecatCycleTime);
  for (int j = 0; j < slaveList.size(); j++)
  {
    if (slaveList[j]->getType() == "IMC")
    {
      IMC* tmpIMCSlave = (IMC*)slaveList[j];
      tmpIMCSlave->PDOmapping();
      tmpIMCSlave->StartupSDO(ecatCycleTime);
    }
  }

  // Configure the EtherCAT message structure depending on the PDO mapping of all the slaves
  ec_config_map(&IOmap);

  ec_configdc();

  // Wait for all slaves to reach SAFE_OP state
  ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

  printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1],
         ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

  printf("Request operational state for all slaves\n");
  expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
  printf("Calculated workcounter %d\n", expectedWKC);
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
  } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

  if (ec_slave[0].state == EC_STATE_OPERATIONAL)
  {
    // All slaves in operational state
    printf("Operational state reached for all slaves.\n");
    inOP = true;
  }
  else
  {
    // Not all slaves in operational state
    ROS_ERROR("Not all slaves reached operational state");
    ec_readstate();
    for (int i = 1; i <= ec_slavecount; i++)
    {
      if (ec_slave[i].state != EC_STATE_OPERATIONAL)
      {
        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n", i, ec_slave[i].state, ec_slave[i].ALstatuscode,
               ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
      }
    }
  }
}

EthercatMaster::~EthercatMaster()
{
  inOP = false;
  printf("Deconstructing EthercatMaster object\n");
  printf("Request init state for all slaves\n");
  ec_slave[0].state = EC_STATE_INIT;
  ec_writestate(0);
  printf("Closing EtherCAT\n");
  ec_close();
  printf("Shutting down ROS\n");
  ros::shutdown();
}

void EthercatMaster::SendProcessData()
{
  ec_send_processdata();
}

int EthercatMaster::ReceiveProcessData()
{
  return ec_receive_processdata(EC_TIMEOUTRET);
}

void EthercatMaster::PublishProcessData()
{
  // Publish for all slaves except the master (slave 0)
  for (int i = 1; i < slaveList.size(); i++)
  {
    // Todo: Add other slave publisher implementations
    std::string slaveType = slaveList[i]->getType();
    if (slaveType == "TEMPLATEGES")
    {
      TemplateGES* tmpTemplateGES = (TemplateGES*)slaveList[i];
      tmpTemplateGES->publish();
    }
    else if (slaveType == "IMC")
    {
    }
    else if (slaveType == "PDB")
    {
    }
    else if (slaveType == "GES")
    {
    }
    else
    {
      printf("Error when getting GES data! Unknown GES name\n");
    }
  }
}

void EthercatMaster::MonitorSlaveConnection()
{
  // Todo: refactor this
  ethercat_safety::monitor_slave_connection();
}
