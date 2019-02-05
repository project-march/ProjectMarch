//
// Created by Martijn on 4-2-19.
//

#include "EthercatMaster.h"
#include "launch_parameters.h"
#include "ethercat_SDO.h"
#include "ethercat_safety.h"

extern "C" {
#include "ethercat.h"
}

// Constructor
EthercatMaster::EthercatMaster(ros::NodeHandle nh, std::vector<Slave> slaveList)
{
  // Get the name of the network interface (if) over which SOEM will be run from the launch file
  nh.getParam(ros::this_node::getName() + "/ifname", ifname);

  inOP = false;

  printf("Starting ethercat\n");

  // Initialise SOEM, bind socket to ifname
  if (!ec_init(ifname.c_str()))
  {
    ROS_ERROR("No socket connection on %s", ifname.c_str());
    ros::shutdown();
  }
  printf("ec_init on %s succeeded.\n", ifname.c_str());

  // Find and auto-config slaves
  if (ec_config_init(FALSE) <= 0)
  {
    ROS_ERROR("No slaves found, shutting down");
    ec_close();
    ros::shutdown();
  }
  printf("%d slaves found and configured.\n", ec_slavecount);

  // Request and wait for slaves to be in preOP state
  // Todo: Change to 0
  ec_statecheck(1, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 4);

  // Over all iMotionCubes:
  //    Apply PDO-mapping while in PreOP state
  //    Send initialization SDO messages
  for (int j = 0; j < *LaunchParameters::get_number_of_joints(); j++)
  {
    int slaveNumber = LaunchParameters::get_slave_number((*LaunchParameters::get_list_of_joints())[j]);
    this->PDOmapping(slaveNumber);
    this->StartupSDO(slaveNumber);
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
    // Call destructor
    this->~EthercatMaster();
  }
}

EthercatMaster::~EthercatMaster()
{
  inOP = false;
  printf("Deconstructing EthercatMaster object\n");
  printf("Request init state for all slaves\n");
  ec_slave[0].state = EC_STATE_INIT;
  ec_writestate(0);
  ec_close();
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
  for (vector<string>::iterator i = LaunchParameters::get_list_of_GES()->begin();
       i != LaunchParameters::get_list_of_GES()->end(); ++i)
  {
    // Todo: Add other slave types
    // Todo: Make slaves a class

    if (*i == "TEMPLATEGES")
    {
      int8 ret_value = this->GetByte(*i, 0);
      printf("Returned value: %d\n", ret_value);
    }
    else if (*i == "IMC")
    {
    }
    else if (*i == "PDB")
    {
    }
    else if (*i == "GES")
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

void EthercatMaster::SetByte(std::string slaveName, uint8 offset, int8 byte)
{
  // Todo: make this a general-purpose set output method
  union bit8 unionbyte;
  unionbyte.i = byte;
  uint16 slaveNumber = LaunchParameters::get_slave_number(slaveName);
  set_output_bit8(slaveNumber, offset, unionbyte);
}

int8 EthercatMaster::GetByte(std::string slaveName, uint8 offset)
{
  // Todo: make this a general-purpose get input method
  uint16 slaveNumber = LaunchParameters::get_slave_number(slaveName);
  return get_input_bit8(slaveNumber, offset).i;
}

int EthercatMaster::PDOmapping(int slave)
{
  printf("PDO mapping START!\n");

  bool success = true;

  //----------------------------------------

  // clear sm pdos
  success &= sdo_bit8(slave, 0x1C12, 0, 0);
  success &= sdo_bit8(slave, 0x1C13, 0, 0);

  //----------------------------------------

  // clear 1A00 pdo entries
  success &= sdo_bit32(slave, 0x1A00, 0, 0);
  // download 1A00 pdo entries
  success &= sdo_bit32(slave, 0x1A00, 1, 0x60410010);
  success &= sdo_bit32(slave, 0x1A00, 2, 0x60640020);
  success &= sdo_bit32(slave, 0x1A00, 3, 0x20000010);
  // download 1A00 pdo count: 3
  success &= sdo_bit32(slave, 0x1A00, 0, 3);
  //--------------------

  // clear 1A01 pdo entries
  success &= sdo_bit32(slave, 0x1A01, 0, 0);
  // download 1A01 pdo entries
  success &= sdo_bit32(slave, 0x1A01, 1, 0x20020010);
  success &= sdo_bit32(slave, 0x1A01, 2, 0x20550010);
  success &= sdo_bit32(slave, 0x1A01, 3, 0x20580010);
  // download 1A01 pdo count: 4
  success &= sdo_bit32(slave, 0x1A01, 0, 3);

  // clear 1A02 pdo entries
  success &= sdo_bit32(slave, 0x1A02, 0, 0);
  // download 1A02 pdo entries
  success &= sdo_bit32(slave, 0x1A02, 1, 0x60770010);
  success &= sdo_bit32(slave, 0x1A02, 2, 0x207f0010);
  success &= sdo_bit32(slave, 0x1A02, 3, 0x20880020);
  // download 1A02 pdo count: 1
  success &= sdo_bit32(slave, 0x1A02, 0, 3);
  // clear 1A03 pdo entries
  success &= sdo_bit32(slave, 0x1A03, 0, 0);

  //----------------------------------------

  // clear 1600 pdo entries
  success &= sdo_bit32(slave, 0x1600, 0, 0);
  // download 1600 pdo entries
  success &= sdo_bit32(slave, 0x1600, 1, 0x60400010);
  success &= sdo_bit32(slave, 0x1600, 2, 0x607A0020);
  // download 1600 pdo count: 2
  success &= sdo_bit32(slave, 0x1600, 0, 2);

  //--------------------

  // clear 1601 pdo entries
  success &= sdo_bit32(slave, 0x1601, 0, 0);
  // clear 1602 pdo entries
  success &= sdo_bit32(slave, 0x1602, 0, 0);
  // clear 1603 pdo entries
  success &= sdo_bit32(slave, 0x1603, 0, 0);

  //----------------------------------------

  // download 1C12:01 index
  success &= sdo_bit16(slave, 0x1C12, 1, 0x1600);
  // download 1C12 counter
  success &= sdo_bit8(slave, 0x1C12, 0, 1);

  //--------------------

  // download 1C13:01 index
  success &= sdo_bit16(slave, 0x1C13, 1, 0x1A00);
  success &= sdo_bit16(slave, 0x1C13, 2, 0x1A01);
  success &= sdo_bit16(slave, 0x1c13, 3, 0x1A02);
  // download 1C13 counter
  success &= sdo_bit8(slave, 0x1C13, 0, 3);

  //----------------------------------------

  printf(success ? "true\n" : "false\n");

  return success;
}

int EthercatMaster::StartupSDO(int slave)
{
  bool success = true;
  printf("Startup SDO\n");

  //----------------------------------------
  // Start writing to sdo
  //----------------------------------------

  // mode of operation
  success &= sdo_bit8(slave, 0x6060, 0, 8);

  // position dimension index
  success &= sdo_bit8(slave, 0x608A, 0, 1);

  // position factor -- scaling factor numerator
  success &= sdo_bit32(slave, 0x6093, 1, 1);
  // position factor -- scaling factor denominator
  success &= sdo_bit32(slave, 0x6093, 2, 1);

  // set the ethercat rate of encoder in form x*10^y
  success &= sdo_bit8(slave, 0x60C2, 1, *LaunchParameters::get_ethercat_cycle_time());
  success &= sdo_bit8(slave, 0x60C2, 2, -3);

  printf(success ? "true\n" : "false\n");

  return success;
}
