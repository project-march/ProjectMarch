#include <ros/ros.h>

#include <march_hardware/EtherCAT/EthercatSDO.h>

#include <march_hardware/IMotionCube.h>

IMotionCube::IMotionCube(int slaveIndex)
{
  this->slaveIndex = slaveIndex;
}

void IMotionCube::initialize()
{
  //    TODO(Martijn Isha) change ecat cycle time magic number
  PDOmapping();
  StartupSDO(200);
}

// Map Process Data Object by sending SDOs to the IMC
bool IMotionCube::PDOmapping()
{
  ROS_DEBUG("PDO mapping START!\n");

  bool success = true;

  //----------------------------------------

  // clear sm pdos
  success &= sdo_bit8(slaveIndex, 0x1C12, 0, 0);
  success &= sdo_bit8(slaveIndex, 0x1C13, 0, 0);

  //----------------------------------------

  // clear 1A00 pdo entries
  success &= sdo_bit32(slaveIndex, 0x1A00, 0, 0);
  // download 1A00 pdo entries
  success &= sdo_bit32(slaveIndex, 0x1A00, 1, 0x60410010);
  success &= sdo_bit32(slaveIndex, 0x1A00, 2, 0x60640020);
  success &= sdo_bit32(slaveIndex, 0x1A00, 3, 0x20000010);
  // download 1A00 pdo count: 3
  success &= sdo_bit32(slaveIndex, 0x1A00, 0, 3);
  //--------------------

  // clear 1A01 pdo entries
  success &= sdo_bit32(slaveIndex, 0x1A01, 0, 0);
  // download 1A01 pdo entries
  success &= sdo_bit32(slaveIndex, 0x1A01, 1, 0x20020010);
  success &= sdo_bit32(slaveIndex, 0x1A01, 2, 0x20550010);
  success &= sdo_bit32(slaveIndex, 0x1A01, 3, 0x20580010);
  // download 1A01 pdo count: 4
  success &= sdo_bit32(slaveIndex, 0x1A01, 0, 3);

  // clear 1A02 pdo entries
  success &= sdo_bit32(slaveIndex, 0x1A02, 0, 0);
  // download 1A02 pdo entries
  success &= sdo_bit32(slaveIndex, 0x1A02, 1, 0x60770010);
  success &= sdo_bit32(slaveIndex, 0x1A02, 2, 0x207f0010);
  success &= sdo_bit32(slaveIndex, 0x1A02, 3, 0x20880020);
  // download 1A02 pdo count: 1
  success &= sdo_bit32(slaveIndex, 0x1A02, 0, 3);
  // clear 1A03 pdo entries
  success &= sdo_bit32(slaveIndex, 0x1A03, 0, 0);

  //----------------------------------------

  // clear 1600 pdo entries
  success &= sdo_bit32(slaveIndex, 0x1600, 0, 0);
  // download 1600 pdo entries
  success &= sdo_bit32(slaveIndex, 0x1600, 1, 0x60400010);
  success &= sdo_bit32(slaveIndex, 0x1600, 2, 0x607A0020);
  // download 1600 pdo count: 2
  success &= sdo_bit32(slaveIndex, 0x1600, 0, 2);

  //--------------------

  // clear 1601 pdo entries
  success &= sdo_bit32(slaveIndex, 0x1601, 0, 0);
  // clear 1602 pdo entries
  success &= sdo_bit32(slaveIndex, 0x1602, 0, 0);
  // clear 1603 pdo entries
  success &= sdo_bit32(slaveIndex, 0x1603, 0, 0);

  //----------------------------------------

  // download 1C12:01 index
  success &= sdo_bit16(slaveIndex, 0x1C12, 1, 0x1600);
  // download 1C12 counter
  success &= sdo_bit8(slaveIndex, 0x1C12, 0, 1);

  //--------------------

  // download 1C13:01 index
  success &= sdo_bit16(slaveIndex, 0x1C13, 1, 0x1A00);
  success &= sdo_bit16(slaveIndex, 0x1C13, 2, 0x1A01);
  success &= sdo_bit16(slaveIndex, 0x1c13, 3, 0x1A02);
  // download 1C13 counter
  success &= sdo_bit8(slaveIndex, 0x1C13, 0, 3);

  //----------------------------------------

  ROS_DEBUG(success ? "true\n" : "false\n");

  return success;
}

// Set configuration parameters to the IMC
bool IMotionCube::StartupSDO(int ecatCycleTime)
{
  bool success = true;
  ROS_DEBUG("Startup SDO\n");

  //----------------------------------------
  // Start writing to sdo
  //----------------------------------------

  // mode of operation
  success &= sdo_bit8(slaveIndex, 0x6060, 0, 8);

  // position dimension index
  success &= sdo_bit8(slaveIndex, 0x608A, 0, 1);

  // position factor -- scaling factor numerator
  success &= sdo_bit32(slaveIndex, 0x6093, 1, 1);
  // position factor -- scaling factor denominator
  success &= sdo_bit32(slaveIndex, 0x6093, 2, 1);

  // set the ethercat rate of encoder in form x*10^y
  success &= sdo_bit8(slaveIndex, 0x60C2, 1, ecatCycleTime);
  success &= sdo_bit8(slaveIndex, 0x60C2, 2, -3);

  ROS_DEBUG(success ? "true\n" : "false\n");

  return success;
}

float IMotionCube::getAngle()
{
  // TODO(Martijn) read from ethercat
  return 0;
}

int IMotionCube::getSlaveIndex() {
    return slaveIndex;
}
