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
void IMotionCube::initialize()
{
  // TODO(Martijn Isha) change ecat cycle time magic number
  PDOmapping();
  StartupSDO(4);
}

// Map Process Data Object by sending SDOs to the IMC
bool IMotionCube::PDOmapping()
{
  ROS_INFO("Start PDO mapping of IMC %d", this->slaveIndex);
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

  ROS_INFO_STREAM("PDO mapping was a " << (success ? "success" : "failure"));

  return success;
}

// Set configuration parameters to the IMC
bool IMotionCube::StartupSDO(uint8 ecatCycleTime)
{
  bool success = true;
  ROS_INFO("Start SDO writing to IMC %d", this->slaveIndex);

  // mode of operation
  success &= sdo_bit8(slaveIndex, 0x6060, 0, 8);

  // position dimension index
  success &= sdo_bit8(slaveIndex, 0x608A, 0, 1);

  //  Todo(Isha) implement position factor scaling

  // position factor -- scaling factor numerator
  success &= sdo_bit32(slaveIndex, 0x6093, 1, 1);
  // position factor -- scaling factor denominator
  success &= sdo_bit32(slaveIndex, 0x6093, 2, 1);

  // set the ethercat rate of encoder in form x*10^y
  success &= sdo_bit8(slaveIndex, 0x60C2, 1, ecatCycleTime);
  success &= sdo_bit8(slaveIndex, 0x60C2, 2, -3);


  ROS_INFO_STREAM("SDO writing was a " << (success ? "success" : "failure"));

  return success;
}

void IMotionCube::actuateRad(float targetRad){
    this->actuateIU(this->encoder.RadtoIU(targetRad));
}

void IMotionCube::actuateIU(int targetIU)
{
  ROS_INFO("Trying to move to position %i", targetIU);
  if (!this->encoder.isValidTargetPositionIU(targetIU))
  {
    ROS_ERROR("Position %i is invalid.", targetIU);
    return;
  }

  union bit32 targetPosition;
  targetPosition.i = targetIU;

  uint8 targetPositionLocation = 2;

  ROS_INFO("Trying to actuate slave %d, soem location %d to targetposition %d", this->slaveIndex, targetPositionLocation,
           targetPosition.i);
  set_output_bit32(this->slaveIndex, targetPositionLocation, targetPosition);

}

float IMotionCube::getAngleRad()
{
  return this->encoder.getAngleRad();
}

uint16 IMotionCube::getMotionError()
{
  return get_input_bit16(this->slaveIndex, 6).ui;
}

uint16 IMotionCube::getDetailedError()
{
  return get_input_bit16(this->slaveIndex, 8).ui;
}

uint16 IMotionCube::getStatusWord()
{
  uint16 statusWord = get_input_bit16(this->slaveIndex, 0).ui;
//  if (get_bit(statusWord, ))

  return statusWord;
}


void IMotionCube::setControlWord(uint16 controlWord)
{
  union bit16 controlwordu;
  controlwordu.i = controlWord;
  set_output_bit16(slaveIndex, 0, controlwordu);
}

void IMotionCube::parseMotionError(int errorCode){
  ROS_WARN_STREAM("Looking up motion error " << std::bitset<16>(errorCode));
  std::vector<std::string> bitDescriptions = {};
  bitDescriptions.push_back("EtherCAT communication error. Reset by a Reset Fault command or by Clear Error in the EtherCAT ® State Machine.");
  bitDescriptions.push_back("Short-circuit. Set when protection is triggered. Reset by a Reset Fault command.");
  bitDescriptions.push_back("Invalid setup data. Set when the EEPROM stored setup data is not valid or not present.");
  bitDescriptions.push_back("Control error (position/speed error too big). Set when protection is triggered. Reset by a Reset Fault command.");
  bitDescriptions.push_back("Communication error. Set when protection is triggered. Reset by a Reset Fault command.");
  bitDescriptions.push_back("Motor position wraps around. Set when protection is triggered. Reset by a Reset Fault command.");
  bitDescriptions.push_back("Positive limit switch active. Set when LSP input is in active state. Reset when LSP input is inactive state");
  bitDescriptions.push_back("Negative limit switch active. Set when LSN input is in active state. Reset when LSN input is inactive state");
  bitDescriptions.push_back("Over current. Set when protection is triggered. Reset by a Reset Fault command");
  bitDescriptions.push_back("I2T protection. Set when protection is triggered. Reset by a Reset Fault command");
  bitDescriptions.push_back("Over temperature motor. Set when protection is triggered. Reset by a Reset Fault command. This protection may be activated if the motor has a PTC or NTC temperature contact.");
  bitDescriptions.push_back("Over temperature drive. Set when protection is triggered. Reset by a Reset Fault command.");
  bitDescriptions.push_back("Over-voltage. Set when protection is triggered. Reset by a Reset Fault command");
  bitDescriptions.push_back("Under-voltage. Set when protection is triggered. Reset by a Reset Fault command");
  bitDescriptions.push_back("Command error. This bit is set in several situations. They can be distinguished either\n"
                            "by the associated emergency code, or in conjunction with other bits:\n"
                            "0xFF03 - Specified homing method not available\n"
                            "0xFF04 - A wrong mode is set in object 6060h, modes_of_operation\n"
                            "0xFF05 - Specified digital I/O line not available\n"
                            "A function is called during the execution of another function (+ set bit 7 of object\n"
                            "6041h, statusword)\n"
                            "Update of operation mode received during a transition\n"
                            "This bit acts just as a warning.");
  bitDescriptions.push_back("Drive disabled due to enable input. Set when enable input is on disable state. Reset when enable input is on enable state");

  for(int i = 0; i<16; i++){
    if(get_bit(errorCode, i) == 1){
        ROS_ERROR_STREAM(bitDescriptions.at(i));
    }
  }

}
bool IMotionCube::goToOperationEnabled(){
    this->setControlWord(128);

    bool switchOnDisabled = false;
    while (!switchOnDisabled){
        this->setControlWord(128);
        int statusWord = this->getStatusWord();
        int switchOnDisabledMask = 0b0000000001001111;
        int switchOnDisabledState = 64;
        switchOnDisabled = (statusWord & switchOnDisabledMask) == switchOnDisabledState;
        ROS_INFO_STREAM_THROTTLE(0.1, "Waiting for switch on disabled: " << std::bitset<16>(statusWord));
    }
    ROS_INFO("Switch on disabled!");

    ROS_INFO("Try to go to 'Ready To switch on'");
    bool readyToSwitchOn = false;
    while (!readyToSwitchOn){
        this->setControlWord(6);
        int statusWord = this->getStatusWord();
        int readyToSwitchOnMask = 0b0000000001101111;
        int readyToSwitchOnState = 33;
        readyToSwitchOn = (statusWord & readyToSwitchOnMask) == readyToSwitchOnState;
        ROS_INFO_STREAM_THROTTLE(0.1, "Waiting for ready to switch on: " << std::bitset<16>(statusWord));
    }
    ROS_INFO("Ready to switch on!");


    bool switchedOn = false;
    while (!switchedOn){
        this->setControlWord(7);
        int statusWord = this->getStatusWord();
        int switchedOnMask = 0b0000000001101111;
        int switchedOnState = 35;
        switchedOn = (statusWord & switchedOnMask) == switchedOnState;
        ROS_INFO_STREAM_THROTTLE(0.1, "Waiting to switch on: " << std::bitset<16>(statusWord));
    }
    ROS_INFO("Switched on!");

    this->actuateIU(this->encoder.getAngleIU());

    bool operationEnabled = false;
    while (!operationEnabled){
        this->setControlWord(15);
        int statusWord = this->getStatusWord();
        int operationEnabledMask = 0b0000000001101111;
        int operationEnabledState = 39;
        operationEnabled = (statusWord & operationEnabledMask) == operationEnabledState;
        ROS_INFO_STREAM_THROTTLE(0.1, "Waiting for operation enabled: " << std::bitset<16>(statusWord));
    }
    ROS_INFO("Operation enabled!");


}


    void IMotionCube::parseStatusWord() {

    int statusWord = this->getStatusWord();
    ROS_WARN("--------------------");
        ROS_WARN_STREAM("Looking up status word " << std::bitset<16>(statusWord));

        if(get_bit(statusWord, 0) == 1){
            ROS_WARN("Axis on. Power stage is enabled. Motor control is performed");
        }

        if(get_bit(statusWord, 1) == 0){
            ROS_WARN("Axis off. Power stage is disabled. Motor control is not performed");
        }

        if(get_bit(statusWord, 2) == 1){
            ROS_WARN("Operation Enabled");
        }

        if(get_bit(statusWord, 3) == 1){
            ROS_WARN("Fault. If set, a fault condition is or was present in the drive.");
        }

        if(get_bit(statusWord, 4) == 1){
            ROS_WARN("Motor supply voltage is present");
        } else {
            ROS_WARN("Motor supply voltage is absent");
        }


        if(get_bit(statusWord, 5) == 0){
            ROS_WARN("Quick Stop. When this bit is zero, the drive is performing a quick stop");
        }
        if(get_bit(statusWord, 6) == 1){
            ROS_WARN("Switch On Disabled.");
        }
        if(get_bit(statusWord, 7) == 1){
            ROS_WARN("Warning. A TML function / homing was called, while another TML function homing is still in execution. The last call is ignored.");
        }
        if(get_bit(statusWord, 8) == 1){
            ROS_WARN("A TML function or homing is executed. Until the function or homing execution ends or is aborted, no other TML function / homing may be called");
        }

        if(get_bit(statusWord, 9) == 1){
            ROS_WARN("Remote – drive parameters may be modified via CAN and the drive will execute the command message.");
        } else {
            ROS_WARN("Remote – drive is in local mode and will not execute the command message.");
        }

        if(get_bit(statusWord, 10) == 1){
            ROS_WARN("Target reached");
        }

        if(get_bit(statusWord, 11) == 1){
            ROS_WARN("Internal Limit Active");
        }

        if(get_bit(statusWord, 12) == 0){
            ROS_WARN("Target position ignored");
        }

        if(get_bit(statusWord, 13) == 1){
            ROS_WARN("Following error");
        }

        if(get_bit(statusWord, 14) == 1){
            ROS_WARN("Last event set has occurred");
        } else{
            ROS_WARN("No event set or the programmed event has not occurred yet");
        }

        if(get_bit(statusWord, 15) == 1){
            ROS_WARN("Axis on. Power stage is enabled. Motor control is performed");
        } else{
            ROS_WARN("Axis off. Power stage is disabled. Motor control is not performed");
        }
        ROS_WARN("--------------------");

    }

bool IMotionCube::get_bit(uint16 value, int index)
{
  return (bool)(value & (1 << index));
}

}  // namespace march4cpp
