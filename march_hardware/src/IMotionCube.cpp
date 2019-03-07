// Copyright 2018 Project March.
#include <ros/ros.h>
#include <bitset>

#include <march_hardware/EtherCAT/EthercatSDO.h>

#include <march_hardware/IMotionCube.h>
#include <unistd.h>
#include <march_hardware/EtherCAT/EthercatIO.h>

namespace march4cpp
{
IMotionCube::IMotionCube(int slaveIndex, Encoder encoder) : Slave(slaveIndex)
{
  this->encoder = encoder;
  this->encoder.setSlaveIndex(this->slaveIndex);
}

// TODO(Martijn Isha) change name to show this is only EtherCAT startup initialization
void IMotionCube::writeInitialSDOs(int ecatCycleTime)
{
  mapPDOs();
  writeInitialSettings(ecatCycleTime);
}

// Map Process Data Object by sending SDOs to the IMC
bool IMotionCube::mapPDOs()
{
  // TODO(Martijn) Refactor this into something more readable and modular

  bool success = true;

  //----------------------------------------

  // clear sm pdos
  success &= sdo_bit8(slaveIndex, 0x1C12, 0, 0);
  success &= sdo_bit8(slaveIndex, 0x1C13, 0, 0);

  //----------------------------------------

  // clear 1A00 pdo entries
  success &= sdo_bit32(slaveIndex, 0x1A00, 0, 0);
  // download 1A00 pdo entries

  // Status word
  success &= sdo_bit32(slaveIndex, 0x1A00, 1, 0x60410010);

  //  Position actual value
  success &= sdo_bit32(slaveIndex, 0x1A00, 2, 0x60640020);

  // Motion error register
  success &= sdo_bit32(slaveIndex, 0x1A00, 3, 0x20000010);

  // download 1A00 pdo count: 3
  success &= sdo_bit32(slaveIndex, 0x1A00, 0, 3);
  //--------------------

  // clear 1A01 pdo entries
  success &= sdo_bit32(slaveIndex, 0x1A01, 0, 0);
  // download 1A01 pdo entries

  //  Detailed error register
  success &= sdo_bit32(slaveIndex, 0x1A01, 1, 0x20020010);
  //  DC-link voltage
  success &= sdo_bit32(slaveIndex, 0x1A01, 2, 0x20550010);
  //  Drive temperature
  success &= sdo_bit32(slaveIndex, 0x1A01, 3, 0x20580010);
  // download 1A01 pdo count: 4
  success &= sdo_bit32(slaveIndex, 0x1A01, 0, 3);

  // clear 1A02 pdo entries
  success &= sdo_bit32(slaveIndex, 0x1A02, 0, 0);
  // download 1A02 pdo entries

  //  Torque actual value
  success &= sdo_bit32(slaveIndex, 0x1A02, 1, 0x60770010);
  //  Current limit
  success &= sdo_bit32(slaveIndex, 0x1A02, 2, 0x207f0010);
  //  Motor position
  success &= sdo_bit32(slaveIndex, 0x1A02, 3, 0x20880020);

  // download 1A02 pdo count: 1
  success &= sdo_bit32(slaveIndex, 0x1A02, 0, 3);
  // clear 1A03 pdo entries
  success &= sdo_bit32(slaveIndex, 0x1A03, 0, 0);

  //----------------------------------------

  // clear 1600 pdo entries
  success &= sdo_bit32(slaveIndex, 0x1600, 0, 0);
  // download 1600 pdo entries
  //  Control word
  success &= sdo_bit32(slaveIndex, 0x1600, 1, 0x60400010);
  //  Target position
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

  if (success)
  {
    ROS_INFO("Successfully mapped PDOs of IMC %d", this->slaveIndex);
  }
  else
  {
    ROS_WARN("PDO mapping of IMC %d failed", this->slaveIndex);
  }

  return success;
}

// Set configuration parameters to the IMC
bool IMotionCube::writeInitialSettings(uint8 ecatCycleTime)
{
  bool success = true;

  // mode of operation
  success &= sdo_bit8(slaveIndex, 0x6060, 0, 8);

  // position dimension index
  success &= sdo_bit8(slaveIndex, 0x608A, 0, 1);

  //  Todo(Isha) implement position factor scaling

  // position factor -- scaling factor numerator
  success &= sdo_bit32(slaveIndex, 0x6093, 1, 1);
  // position factor -- scaling factor denominator
  success &= sdo_bit32(slaveIndex, 0x6093, 2, 1);

  // position limit -- min position
  //  39717 = 0 rad
  success &= sdo_bit32(slaveIndex, 0x607D, 1, 39717);
  // position limit -- max position
  success &= sdo_bit32(slaveIndex, 0x607D, 2, 51000);

  // Quick stop option
  success &= sdo_bit16(slaveIndex, 0x605A, 0, 6);

  // Quick stop deceleration
  success &= sdo_bit32(slaveIndex, 0x6085, 0, 0x7FFFFFFF);

  // set the ethercat rate of encoder in form x*10^y
  success &= sdo_bit8(slaveIndex, 0x60C2, 1, ecatCycleTime);
  success &= sdo_bit8(slaveIndex, 0x60C2, 2, -3);

  if (success)
  {
    ROS_INFO("Successfully wrote initial settings to IMC %d", this->slaveIndex);
  }
  else
  {
    ROS_WARN("Writing initial settings to IMC %d failed", this->slaveIndex);
  }

  return success;
}

void IMotionCube::actuateRad(float targetRad)
{
  this->actuateIU(this->encoder.RadtoIU(targetRad));
}

void IMotionCube::actuateRadFixedSpeed(float targetRad, float radPerSec)
{
  if (radPerSec <= 0)
  {
    ROS_ERROR("Rad per sec must be bigger than 0, given: %f", radPerSec);
    return;
  }
  if (radPerSec > 0.5)
  {
    ROS_ERROR("Rad per sec must be smaller than 0.5, given: %f", radPerSec);
    return;
  }

  float currentRad = this->getAngleRad();
  ROS_INFO("Trying to go from position %f to position %f with speed %f", currentRad, targetRad, radPerSec);
  float distance = targetRad - currentRad;
  int resolution = 250;
  int cycles = std::floor(std::abs(distance) / radPerSec * resolution) + 1;
  for (int i = 0; i < cycles; i++)
  {
    float index = i;
    float calculatedTarget = currentRad + (index / cycles * distance);
    ROS_INFO_STREAM(calculatedTarget);
    usleep(static_cast<__useconds_t>(1000000 / resolution));
    this->actuateRad(calculatedTarget);
  }
}

void IMotionCube::actuateIU(int targetIU)
{
  if (!this->encoder.isValidTargetPositionIU(targetIU))
  {
    ROS_ERROR("Position %i is invalid.", targetIU);
    return;
  }

  union bit32 targetPosition;
  targetPosition.i = targetIU;

  uint8 targetPositionLocation = 2;  // TODO(Isha Martijn) make this dynamic

  ROS_INFO("Trying to actuate slave %d, soem location %d to targetposition %d", this->slaveIndex,
           targetPositionLocation, targetPosition.i);
  set_output_bit32(this->slaveIndex, targetPositionLocation, targetPosition);
}

float IMotionCube::getAngleRad()
{
  return this->encoder.getAngleRad();
}

uint16 IMotionCube::getStatusWord()
{
  return get_input_bit16(this->slaveIndex, 0).ui;
}

uint16 IMotionCube::getMotionError()
{
  return get_input_bit16(this->slaveIndex, 6).ui;
}

uint16 IMotionCube::getDetailedError()
{
  return get_input_bit16(this->slaveIndex, 8).ui;
}

void IMotionCube::setControlWord(uint16 controlWord)
{
  union bit16 controlwordu;
  controlwordu.i = controlWord;
  set_output_bit16(slaveIndex, 0, controlwordu);
}

void IMotionCube::parseStatusWord(uint16 statusWord)
{
  ROS_WARN_STREAM("Looking up Status Word " << std::bitset<16>(statusWord));
  if (get_bit(statusWord, 0) == 1)
  {
    ROS_WARN("\tAxis on. Power stage is enabled. Motor control is performed.");
  }
  else
  {
    ROS_WARN("\tAxis off. Power stage is disabled. Motor control is not performed.");
  }
  if (get_bit(statusWord, 2) == 1)
  {
    ROS_WARN("\tOperation Enabled.");
  }
  if (get_bit(statusWord, 3) == 1)
  {
    ROS_WARN("\tFault. If set, a fault condition is or was present in the drive.");
  }
  if (get_bit(statusWord, 4) == 1)
  {
    ROS_WARN("\tMotor supply voltage is present.");
  }
  else
  {
    ROS_WARN("\tMotor supply voltage is absent.");
  }
  if (get_bit(statusWord, 5) == 0)
  {
    ROS_WARN("\tQuick Stop. When this bit is zero, the drive is performing a quick stop.");
  }
  if (get_bit(statusWord, 6) == 1)
  {
    ROS_WARN("\tSwitch On Disabled.");
  }
  if (get_bit(statusWord, 7) == 1)
  {
    ROS_WARN("\tWarning. A TML function / homing was called, while another TML function homing is still in execution. "
             "The last call is ignored.");
  }
  if (get_bit(statusWord, 8) == 1)
  {
    ROS_WARN("\tA TML function or homing is executed. Until the function or homing execution ends or is aborted, no "
             "other TML function / homing may be called.");
  }
  if (get_bit(statusWord, 9) == 1)
  {
    ROS_WARN("\tRemote - drive parameters may be modified via CAN and the drive will execute the command message.");
  }
  else
  {
    ROS_WARN("\tRemote - drive is in local mode and will not execute the command message.");
  }
  if (get_bit(statusWord, 10) == 1)
  {
    ROS_WARN("\tTarget reached.");
  }
  if (get_bit(statusWord, 11) == 1)
  {
    ROS_WARN("\tInternal Limit Active.");
  }
  if (get_bit(statusWord, 12) == 0)
  {
    ROS_WARN("\tTarget position ignored.");
  }
  if (get_bit(statusWord, 13) == 1)
  {
    ROS_WARN("\tFollowing error.");
  }
  if (get_bit(statusWord, 14) == 1)
  {
    ROS_WARN("\tLast event set has occurred.");
  }
  else
  {
    ROS_WARN("\tNo event set or the programmed event has not occurred yet.");
  }
  if (get_bit(statusWord, 15) == 1)
  {
    ROS_WARN("\tAxis on. Power stage is enabled. Motor control is performed.");
  }
  else
  {
    ROS_WARN("\tAxis off. Power stage is disabled. Motor control is not performed.");
  }
}

void IMotionCube::parseMotionError(uint16 motionError)
{
  ROS_WARN_STREAM("Looking up Motion Error " << std::bitset<16>(motionError));
  std::vector<std::string> bitDescriptions = {};
  bitDescriptions.push_back("\tEtherCAT communication error. Reset by a Reset Fault command or by Clear Error in the "
                            "EtherCAT State Machine.");
  bitDescriptions.push_back("\tShort-circuit. Set when protection is triggered. Reset by a Reset Fault command.");
  bitDescriptions.push_back("\tInvalid setup data. Set when the EEPROM stored setup data is not valid or not present.");
  bitDescriptions.push_back("\tControl error (position/speed error too big). Set when protection is triggered. Reset "
                            "by a Reset Fault command.");
  bitDescriptions.push_back("\tCommunication error. Set when protection is triggered. Reset by a Reset Fault command.");
  bitDescriptions.push_back("\tMotor position wraps around. Set when protection is triggered. Reset by a Reset Fault "
                            "command.");
  bitDescriptions.push_back("\tPositive limit switch active. Set when LSP input is in active state. Reset when LSP "
                            "input is inactive state");
  bitDescriptions.push_back("\tNegative limit switch active. Set when LSN input is in active state. Reset when LSN "
                            "input is inactive state");
  bitDescriptions.push_back("\tOver current. Set when protection is triggered. Reset by a Reset Fault command");
  bitDescriptions.push_back("\tI2T protection. Set when protection is triggered. Reset by a Reset Fault command");
  bitDescriptions.push_back("\tOver temperature motor. Set when protection is triggered. Reset by a Reset Fault "
                            "command. This protection may be activated if the motor has a PTC or NTC temperature "
                            "contact.");
  bitDescriptions.push_back("\tOver temperature drive. Set when protection is triggered. Reset by a Reset Fault "
                            "command.");
  bitDescriptions.push_back("\tOver-voltage. Set when protection is triggered. Reset by a Reset Fault command");
  bitDescriptions.push_back("\tUnder-voltage. Set when protection is triggered. Reset by a Reset Fault command");
  bitDescriptions.push_back(
      "\tCommand error. This bit is set in several situations. They can be distinguished either by the associated "
      "emergency code, or in conjunction with other bits:\n"
      "\t\t0xFF03 - Specified homing method not available\n"
      "\t\t0xFF04 - A wrong mode is set in object 6060h, modes_of_operation\n"
      "\t\t0xFF05 - Specified digital I/O line not available\n"
      "\tA function is called during the execution of another function (+ set bit 7 of object 6041h, statusword).\n"
      "\tUpdate of operation mode received during a transition. This bit acts just as a warning.");
  bitDescriptions.push_back("\tDrive disabled due to enable input. Set when enable input is on disable state. Reset "
                            "when enable input is on enable state");

  for (int i = 0; i < 16; i++)
  {
    if (get_bit(motionError, i) == 1)
    {
      ROS_WARN_STREAM(bitDescriptions.at(i));
    }
  }
}

void IMotionCube::parseDetailedError(uint16 detailedError)
{
  ROS_WARN_STREAM("Looking up Detailed Error " << std::bitset<16>(detailedError));
  std::vector<std::string> bitDescriptions = {};
  bitDescriptions.push_back("\tThe number of nested function calls exceeded the length of TML stack. Last function "
                            "call was ignored.");
  bitDescriptions.push_back("\tA RET/RETI instruction was executed while no function/ISR was active.");
  bitDescriptions.push_back("\tA call to an inexistent homing routine was received.");
  bitDescriptions.push_back("\tA call to an inexistent function was received.");
  bitDescriptions.push_back("\tUPD instruction received while AXISON was executed. The UPD instruction. The UPD "
                            "instruction was ignored and it must be sent again when AXISON is completed.");
  bitDescriptions.push_back("\tCancelable call instruction received while another cancelable function was active.");
  bitDescriptions.push_back("\tPositive software limit switch is active.");
  bitDescriptions.push_back("\tNegative software limit switch is active.");
  bitDescriptions.push_back("\tS-curve parameters caused an invalid profile. UPD instruction was ignored.");

  for (int i = 0; i < 9; i++)
  {
    if (get_bit(detailedError, i) == 1)
    {
      ROS_WARN_STREAM(bitDescriptions.at(i));
    }
  }
}

bool IMotionCube::goToOperationEnabled()
{
  this->setControlWord(128);

  ROS_INFO("Try to go to 'Switch on Disabled'");
  bool switchOnDisabled = false;
  while (!switchOnDisabled)
  {
    this->setControlWord(128);
    int statusWord = this->getStatusWord();
    int switchOnDisabledMask = 0b0000000001001111;
    int switchOnDisabledState = 64;
    switchOnDisabled = (statusWord & switchOnDisabledMask) == switchOnDisabledState;
    ROS_INFO_STREAM_THROTTLE(0.1, "Waiting for 'Switch on Disabled': " << std::bitset<16>(statusWord));
  }
  ROS_INFO("Switch on Disabled!");

  ROS_INFO("Try to go to 'Ready to Switch On'");
  bool readyToSwitchOn = false;
  while (!readyToSwitchOn)
  {
    this->setControlWord(6);
    int statusWord = this->getStatusWord();
    int readyToSwitchOnMask = 0b0000000001101111;
    int readyToSwitchOnState = 33;
    readyToSwitchOn = (statusWord & readyToSwitchOnMask) == readyToSwitchOnState;
    ROS_INFO_STREAM_THROTTLE(0.1, "Waiting for 'Ready to Switch On': " << std::bitset<16>(statusWord));
  }
  ROS_INFO("Ready to Switch On!");

  ROS_INFO("Try to go to 'Switched On'");
  bool switchedOn = false;
  while (!switchedOn)
  {
    this->setControlWord(7);
    int statusWord = this->getStatusWord();
    int switchedOnMask = 0b0000000001101111;
    int switchedOnState = 35;
    switchedOn = (statusWord & switchedOnMask) == switchedOnState;
    ROS_INFO_STREAM_THROTTLE(0.1, "Waiting for 'Switched On': " << std::bitset<16>(statusWord));
  }
  ROS_INFO("Switched On!");

  this->actuateIU(this->encoder.getAngleIU());

  ROS_INFO("Try to go to 'Operation Enabled'");
  bool operationEnabled = false;
  while (!operationEnabled)
  {
    this->setControlWord(15);
    int statusWord = this->getStatusWord();
    int operationEnabledMask = 0b0000000001101111;
    int operationEnabledState = 39;
    operationEnabled = (statusWord & operationEnabledMask) == operationEnabledState;
    ROS_INFO_STREAM_THROTTLE(0.1, "Waiting for 'Operation Enabled': " << std::bitset<16>(statusWord));
  }
  ROS_INFO("Operation Enabled!");
}

bool IMotionCube::get_bit(uint16 value, int index)
{
  return static_cast<bool>(value & (1 << index));
}

}  // namespace march4cpp
