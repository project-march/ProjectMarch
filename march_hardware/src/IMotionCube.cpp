// Copyright 2018 Project March.
#include <bitset>
#include <ros/ros.h>

#include <march_hardware/EtherCAT/EthercatSDO.h>

#include <march_hardware/EtherCAT/EthercatIO.h>
#include <march_hardware/IMotionCube.h>
#include <unistd.h>

namespace march4cpp
{
IMotionCube::IMotionCube(int slaveIndex, Encoder encoder) : Slave(slaveIndex), actuationMode("unknown")
{
  this->encoder = encoder;
  this->encoder.setSlaveIndex(this->slaveIndex);
}

void IMotionCube::writeInitialSDOs(int ecatCycleTime)
{
  mapMisoPDOs();
  mapMosiPDOs();
  validateMisoPDOs();
  validateMosiPDOs();
  writeInitialSettings(ecatCycleTime);
}

// Map Process Data Object (PDO) for by sending SDOs to the IMC
// Master In, Slave Out
void IMotionCube::mapMisoPDOs()
{
  PDOmap pdoMapMISO = PDOmap();
  pdoMapMISO.addObject(IMCObjectName::StatusWord);      // Compulsory!
  pdoMapMISO.addObject(IMCObjectName::ActualPosition);  // Compulsory!
  pdoMapMISO.addObject(IMCObjectName::ActualTorque);    // Compulsory!
  pdoMapMISO.addObject(IMCObjectName::MotionErrorRegister);
  pdoMapMISO.addObject(IMCObjectName::DetailedErrorRegister);
  pdoMapMISO.addObject(IMCObjectName::DCLinkVoltage);
  this->misoByteOffsets = pdoMapMISO.map(this->slaveIndex, dataDirection::miso);
}

// Map Process Data Object (PDO) for by sending SDOs to the IMC
// Master Out, Slave In
void IMotionCube::mapMosiPDOs()
{
  PDOmap pdoMapMOSI = PDOmap();
  pdoMapMOSI.addObject(IMCObjectName::ControlWord);  // Compulsory!
  pdoMapMOSI.addObject(IMCObjectName::TargetPosition);
  pdoMapMOSI.addObject(IMCObjectName::TargetTorque);
  this->mosiByteOffsets = pdoMapMOSI.map(this->slaveIndex, dataDirection::mosi);
}

// Checks if the compulsory MISO PDO objects are mapped
void IMotionCube::validateMisoPDOs()
{
  ROS_ASSERT_MSG(this->misoByteOffsets.count(IMCObjectName::StatusWord) == 1, "StatusWord not mapped");
  ROS_ASSERT_MSG(this->misoByteOffsets.count(IMCObjectName::ActualPosition) == 1, "ActualPosition not mapped");
  ROS_ASSERT_MSG(this->mosiByteOffsets.count(IMCObjectName::TargetTorque) == 1, "TargetTorque not mapped");

  if (this->actuationMode == ActuationMode::position)
  {
    ROS_ASSERT_MSG(this->mosiByteOffsets.count(IMCObjectName::TargetPosition) == 1, "TargetPosition not mapped");
  }
}

// Checks if the compulsory MOSI PDO objects are mapped
void IMotionCube::validateMosiPDOs()
{
  ROS_ASSERT_MSG(this->mosiByteOffsets.count(IMCObjectName::ControlWord) == 1, "ControlWord not mapped");
}

// Set configuration parameters to the IMC
void IMotionCube::writeInitialSettings(uint8 ecatCycleTime)
{
  ROS_DEBUG("IMotionCube::writeInitialSettings");
  bool success = true;
  // sdo_bit32(slaveIndex, address, subindex, value);

  if (this->actuationMode == ActuationMode::unknown)
  {
    throw std::runtime_error("Cannot write initial settings to IMotionCube as it has actuation mode of unknown");
  }

  // mode of operation
  success &= sdo_bit8(slaveIndex, 0x6060, 0, this->actuationMode.toModeNumber());

  // position dimension index
  success &= sdo_bit8(slaveIndex, 0x608A, 0, 1);

  // position factor -- scaling factor numerator
  success &= sdo_bit32(slaveIndex, 0x6093, 1, 1);
  // position factor -- scaling factor denominator
  success &= sdo_bit32(slaveIndex, 0x6093, 2, 1);

  // position limit -- min position
  success &= sdo_bit32(slaveIndex, 0x607D, 1, this->encoder.getLowerSoftLimitIU());
  // position limit -- max position
  success &= sdo_bit32(slaveIndex, 0x607D, 2, this->encoder.getUpperSoftLimitIU());

  // Quick stop option
  success &= sdo_bit16(slaveIndex, 0x605A, 0, 6);

  // Quick stop deceleration
  success &= sdo_bit32(slaveIndex, 0x6085, 0, 0x7FFFFFFF);

  // set the ethercat rate of encoder in form x*10^y
  success &= sdo_bit8(slaveIndex, 0x60C2, 1, ecatCycleTime);
  success &= sdo_bit8(slaveIndex, 0x60C2, 2, -3);

  ROS_ASSERT_MSG(success, "Writing initial settings to IMC %d failed", this->slaveIndex);
}

void IMotionCube::actuateRad(float targetRad)
{
  ROS_ASSERT_MSG(this->actuationMode == ActuationMode::position, "trying to actuate rad, while actuationmode = %s",
                 this->actuationMode.toString().c_str());

  if (std::abs(targetRad - this->getAngleRad()) > 0.27)
  {
    ROS_ERROR("Target %f exceeds max difference of 0.27 from current %f for slave %d", targetRad, this->getAngleRad(),
              this->slaveIndex);
    throw std::runtime_error("Target exceeds max difference of 0.27 from current position");
  }
  this->actuateIU(this->encoder.RadtoIU(targetRad));
}

void IMotionCube::actuateIU(int targetIU)
{
  if (!this->encoder.isValidTargetIU(this->getAngleIU(), targetIU))
  {
    ROS_ERROR("Position %i is invalid for slave %d. (%d, %d)", targetIU, this->slaveIndex,
              this->encoder.getLowerSoftLimitIU(), this->encoder.getUpperSoftLimitIU());
    throw std::runtime_error("Invalid IU actuate command.");
  }

  union bit32 targetPosition;
  targetPosition.i = targetIU;

  if (this->mosiByteOffsets.count(IMCObjectName::TargetPosition) != 1)
  {
    ROS_WARN("TargetPosition not defined in PDO mapping, so can't do actuateIU");
    return;
  }
  uint8 targetPositionLocation = this->mosiByteOffsets[IMCObjectName::TargetPosition];

  ROS_DEBUG("Trying to actuate slave %d, soem location %d to targetposition %d", this->slaveIndex,
            targetPositionLocation, targetPosition.i);
  set_output_bit32(this->slaveIndex, targetPositionLocation, targetPosition);
}

void IMotionCube::actuateTorque(int targetTorque)
{
  ROS_ASSERT_MSG(this->actuationMode == ActuationMode::torque, "trying to actuate torque, while actuationmode = %s",
                 this->actuationMode.toString().c_str());

  // The targetTorque must not exceed the value of 27300 IU, this is 25 A. This value could be increased in the future
  // (to 30A) with good reasoning.
  ROS_ASSERT_MSG(targetTorque < 27300, "Torque of %d is too high.", targetTorque);

  union bit16 targetTorqueStruct;
  targetTorqueStruct.i = targetTorque;

  if (this->mosiByteOffsets.count(IMCObjectName::TargetTorque) != 1)
  {
    ROS_WARN("TargetTorque not defined in PDO mapping, so can't do actuateTorque");
    return;
  }
  uint8 targetTorqueLocation = this->mosiByteOffsets[IMCObjectName::TargetTorque];

  ROS_DEBUG("Trying to actuate slave %d, soem location %d with target torque %d", this->slaveIndex,
            targetTorqueLocation, targetTorqueStruct.i);
  set_output_bit16(this->slaveIndex, targetTorqueLocation, targetTorqueStruct);
}

float IMotionCube::getAngleRad()
{
  ROS_ASSERT_MSG(this->misoByteOffsets.count(IMCObjectName::ActualPosition) == 1, "ActualPosition not defined in PDO "
                                                                                  "mapping, so can't get angle");
  return this->encoder.getAngleRad(this->misoByteOffsets[IMCObjectName::ActualPosition]);
}

float IMotionCube::getTorque()
{
  ROS_ASSERT_MSG(this->misoByteOffsets.count(IMCObjectName::ActualTorque) == 1, "ActualTorque not defined in PDO "
                                                                                "mapping, so can't get torque");
  union bit16 return_byte = get_input_bit16(this->slaveIndex, this->misoByteOffsets[IMCObjectName::ActualTorque]);
  ROS_DEBUG("Actual Torque read: %d", return_byte.i);
  return return_byte.i;
}

int IMotionCube::getAngleIU()
{
  ROS_ASSERT_MSG(this->misoByteOffsets.count(IMCObjectName::ActualPosition) == 1, "ActualPosition not defined in PDO "
                                                                                  "mapping, so can't get angle");
  return this->encoder.getAngleIU(this->misoByteOffsets[IMCObjectName::ActualPosition]);
}

uint16 IMotionCube::getStatusWord()
{
  ROS_ASSERT_MSG(this->misoByteOffsets.count(IMCObjectName::StatusWord) == 1, "StatusWord not defined in PDO "
                                                                              "mapping, so can't get status word");
  return get_input_bit16(this->slaveIndex, this->misoByteOffsets[IMCObjectName::StatusWord]).ui;
}

uint16 IMotionCube::getMotionError()
{
  if (this->misoByteOffsets.count(IMCObjectName::MotionErrorRegister) != 1)
  {
    ROS_WARN("MotionErrorRegister not defined in PDO mapping, so can't read it");
    return 0xFFFF;  // Not fatal, so can return
  }
  return get_input_bit16(this->slaveIndex, this->misoByteOffsets[IMCObjectName::MotionErrorRegister]).ui;
}

uint16 IMotionCube::getDetailedError()
{
  if (this->misoByteOffsets.count(IMCObjectName::DetailedErrorRegister) != 1)
  {
    ROS_WARN("DetailedErrorRegister not defined in PDO mapping, so can't read it");
    return 0xFFFF;  // Not fatal, so can return
  }
  return get_input_bit16(this->slaveIndex, this->misoByteOffsets[IMCObjectName::DetailedErrorRegister]).ui;
}

float IMotionCube::getMotorCurrent()
{
  const float PEAK_CURRENT = 40.0;            // Peak current of iMC drive
  const float IU_CONVERSION_CONST = 65520.0;  // Conversion parameter, see Technosoft CoE programming manual
  if (this->misoByteOffsets.count(IMCObjectName::ActualTorque) != 1)
  {
    ROS_WARN("ActualTorque not defined in PDO mapping, so can't read it");
    return 0xFFFF;  // Not fatal, so can return
  }
  int16_t motorCurrentIU = get_input_bit16(this->slaveIndex, this->misoByteOffsets[IMCObjectName::ActualTorque]).i;
  float motorCurrentA = (2.0 * PEAK_CURRENT / IU_CONVERSION_CONST) *
                        motorCurrentIU;  // Conversion to Amp, see Technosoft CoE programming manual
  return motorCurrentA;
}

float IMotionCube::getMotorVoltage()
{
  const float V_DC_MAX_MEASURABLE = 102.3;    // maximum measurable DC voltage found in EMS Setup/Drive info button
  const float IU_CONVERSION_CONST = 65520.0;  // Conversion parameter, see Technosoft CoE programming manual
  if (this->misoByteOffsets.count(IMCObjectName::DCLinkVoltage) != 1)
  {
    ROS_WARN("DC-link Voltage not defined in PDO mapping, so can't read it");
    return 0xFFFF;  // Not fatal, so can return
  }
  uint16_t motorVoltageIU = get_input_bit16(this->slaveIndex, this->misoByteOffsets[IMCObjectName::DCLinkVoltage]).ui;
  float motorVoltageV = (V_DC_MAX_MEASURABLE / IU_CONVERSION_CONST) *
                        motorVoltageIU;  // Conversion to Volt, see Technosoft CoE programming manual
  return motorVoltageV;
}

void IMotionCube::setControlWord(uint16 controlWord)
{
  if (this->mosiByteOffsets.count(IMCObjectName::ControlWord) != 1)
  {
    ROS_FATAL("ControlWord not defined in PDO mapping, so can't set Control Word");
    throw std::exception();
  }
  union bit16 controlwordu;
  controlwordu.i = controlWord;
  set_output_bit16(slaveIndex, this->mosiByteOffsets[IMCObjectName::ControlWord], controlwordu);
}

std::string IMotionCube::parseStatusWord(uint16 statusWord)
{
  std::string wordDescription = "";
  if (get_bit(statusWord, 0) == 1)
  {
    wordDescription += "Axis on. Power stage is enabled. Motor control is performed. ";
  }
  else
  {
    wordDescription += "Axis off. Power stage is disabled. Motor control is not performed. ";
  }
  if (get_bit(statusWord, 2) == 1)
  {
    wordDescription += "Operation Enabled. ";
  }
  if (get_bit(statusWord, 3) == 1)
  {
    wordDescription += "Fault. If set, a fault condition is or was present in the drive. ";
  }
  if (get_bit(statusWord, 4) == 1)
  {
    wordDescription += "Motor supply voltage is present. ";
  }
  else
  {
    wordDescription += "Motor supply voltage is absent. ";
  }
  if (get_bit(statusWord, 5) == 0)
  {
    wordDescription += "Quick Stop. When this bit is zero, the drive is performing a quick stop. ";
  }
  if (get_bit(statusWord, 6) == 1)
  {
    wordDescription += "Switch On Disabled. ";
  }
  if (get_bit(statusWord, 7) == 1)
  {
    wordDescription += "A TML function  was called, while another TML function is still in execution. ";
  }
  if (get_bit(statusWord, 8) == 1)
  {
    wordDescription += "A TML function or homing is executed. ";
  }
  if (get_bit(statusWord, 9) == 1)
  {
    wordDescription += "Remote - drive parameters may be modified via CAN. ";
  }
  else
  {
    wordDescription += "Remote - drive is in local mode and will not execute the command message. ";
  }
  if (get_bit(statusWord, 10) == 1)
  {
    wordDescription += "Target reached. ";
  }
  if (get_bit(statusWord, 11) == 1)
  {
    wordDescription += "Internal Limit Active. ";
  }
  if (get_bit(statusWord, 12) == 0)
  {
    wordDescription += "Target position ignored. ";
  }
  if (get_bit(statusWord, 13) == 1)
  {
    wordDescription += "Following error. ";
  }
  if (get_bit(statusWord, 14) == 1)
  {
    wordDescription += "Last event set has occurred. ";
  }
  else
  {
    wordDescription += "No event set or the programmed event has not occurred yet. ";
  }
  if (get_bit(statusWord, 15) == 1)
  {
    wordDescription += "Axis on. Power stage is enabled. Motor control is performed. ";
  }
  else
  {
    wordDescription += "Axis off. Power stage is disabled. Motor control is not performed. ";
  }
}

IMCState IMotionCube::getState(uint16 statusWord)
{
  uint16 fiveBitMask = 0b0000000001001111;
  uint16 sixBitMask = 0b0000000001101111;

  uint16 notReadyToSwitchOn = 0b0000000000000000;
  uint16 switchOnDisabled = 0b0000000001000000;
  uint16 readyToSwitchOn = 0b0000000000100001;
  uint16 switchedOn = 0b0000000000100011;
  uint16 operationEnabled = 0b0000000000100111;
  uint16 quickStopActive = 0b0000000000000111;
  uint16 faultReactionActive = 0b0000000000001111;
  uint16 fault = 0b0000000000001000;

  uint16 statusWordFiveBitMasked = (statusWord & fiveBitMask);
  uint16 statusWordSixBitMasked = (statusWord & sixBitMask);

  if (statusWordFiveBitMasked == notReadyToSwitchOn)
  {
    return IMCState::notReadyToSwitchOn;
  }
  else if (statusWordFiveBitMasked == switchOnDisabled)
  {
    return IMCState::switchOnDisabled;
  }
  else if (statusWordSixBitMasked == readyToSwitchOn)
  {
    return IMCState::readyToSwitchOn;
  }
  else if (statusWordSixBitMasked == switchedOn)
  {
    return IMCState::switchedOn;
  }
  else if (statusWordSixBitMasked == operationEnabled)
  {
    return IMCState::operationEnabled;
  }
  else if (statusWordSixBitMasked == quickStopActive)
  {
    return IMCState::quickStopActive;
  }
  else if (statusWordFiveBitMasked == faultReactionActive)
  {
    return IMCState::faultReactionActive;
  }
  else if (statusWordFiveBitMasked == fault)
  {
    return IMCState::fault;
  }
  else
  {
    return IMCState::unknown;
  }
}

std::string IMotionCube::parseMotionError(uint16 motionError)
{
  std::vector<std::string> bitDescriptions = {};
  bitDescriptions.push_back("EtherCAT communication error. ");
  bitDescriptions.push_back("Short-circuit. ");
  bitDescriptions.push_back("Invalid setup (EEPROM) data. ");
  bitDescriptions.push_back("Control error (position/speed error too big). ");
  bitDescriptions.push_back("Communication error. ");
  bitDescriptions.push_back("Motor position wraps around. ");
  bitDescriptions.push_back("Positive limit switch. ");
  bitDescriptions.push_back("Negative limit switch. ");
  bitDescriptions.push_back("Over-current. ");
  bitDescriptions.push_back("I2T protection. ");
  bitDescriptions.push_back("Over-temperature motor. ");
  bitDescriptions.push_back("Over-temperature drive. ");
  bitDescriptions.push_back("Over-voltage. ");
  bitDescriptions.push_back("Under-voltage. ");
  bitDescriptions.push_back("Command error. ");
  bitDescriptions.push_back("Drive disabled (Emergency button connector not shorted). ");

  std::string errorDescription = "";
  for (int i = 0; i < 16; i++)
  {
    if (get_bit(motionError, i) == 1)
    {
      errorDescription = errorDescription + bitDescriptions.at(i);
    }
  }
  return errorDescription;
}

std::string IMotionCube::parseDetailedError(uint16 detailedError)
{
  std::vector<std::string> bitDescriptions = {};
  bitDescriptions.push_back("TML stack overflow. ");
  bitDescriptions.push_back("TML stack underflow. ");
  bitDescriptions.push_back("Homing not available. ");
  bitDescriptions.push_back("Function not available. ");
  bitDescriptions.push_back("UPD ignored. ");
  bitDescriptions.push_back("Cancelable call ignored. ");
  bitDescriptions.push_back("Positive software limit switch is active. ");
  bitDescriptions.push_back("Negative software limit switch is active. ");
  bitDescriptions.push_back("Invalid S-curve profile. ");

  std::string errorDescription = "";
  for (int i = 0; i < 9; i++)
  {
    if (get_bit(detailedError, i) == 1)
    {
      errorDescription = errorDescription + bitDescriptions.at(i);
    }
  }
  return errorDescription;
}

bool IMotionCube::goToTargetState(IMotionCubeTargetState targetState)
{
  ROS_INFO("\tTry to go to '%s'", targetState.getDescription().c_str());
  while (!targetState.isReached(this->getStatusWord()))
  {
    this->setControlWord(targetState.getControlWord());
    ROS_INFO_THROTTLE(0.5, "\tWaiting for '%s': %s", targetState.getDescription().c_str(),
                      std::bitset<16>(this->getStatusWord()).to_string().c_str());
  }
  ROS_INFO("\tReached '%s'!", targetState.getDescription().c_str());
}

bool IMotionCube::goToOperationEnabled()
{
  this->setControlWord(128);

  this->goToTargetState(IMotionCubeTargetState::SWITCH_ON_DISABLED);
  this->goToTargetState(IMotionCubeTargetState::READY_TO_SWITCH_ON);
  this->goToTargetState(IMotionCubeTargetState::SWITCHED_ON);
  // If ActualPosition is not defined in PDOmapping, a fatal error is thrown
  // because of safety reasons
  ROS_ASSERT_MSG(this->misoByteOffsets.count(IMCObjectName::ActualPosition) == 1, "ActualPosition not defined in PDO "
                                                                                  "mapping, so can't get angle");

  int angleRead = this->encoder.getAngleIU(this->misoByteOffsets[IMCObjectName::ActualPosition]);
  //  If the encoder is functioning correctly, move the joint to its current
  //  position. Otherwise shutdown
  if (this->encoder.isWithinHardLimitsIU(angleRead) && angleRead != 0)
  {
    if (this->actuationMode == ActuationMode::position)
    {
      this->actuateIU(angleRead);
    }
    if (this->actuationMode == ActuationMode::torque)
    {
      this->actuateTorque(0);
    }
  }
  else
  {
    ROS_FATAL("Encoder of iMotionCube (with slaveindex %d) is not functioning properly, read value %d, min value "
              "is %d, max value is %d. Shutting down",
              this->slaveIndex, angleRead, this->encoder.getLowerHardLimitIU(), this->encoder.getUpperHardLimitIU());
    throw std::domain_error("Encoder is not functioning properly");
  }

  this->goToTargetState(IMotionCubeTargetState::OPERATION_ENABLED);
}

bool IMotionCube::resetIMotionCube()
{
  this->setControlWord(0);
  ROS_DEBUG("Slave: %d, Try to reset IMC", this->slaveIndex);
  sdo_bit16(slaveIndex, 0x2080, 0, 1);
}

ActuationMode IMotionCube::getActuationMode() const
{
  return this->actuationMode;
}

void IMotionCube::setActuationMode(ActuationMode mode)
{
  if (this->actuationMode != ActuationMode::unknown)
  {
    throw std::runtime_error("Cannot change actuation mode at runtime");
  }
  this->actuationMode = mode;
}

bool IMotionCube::get_bit(uint16 value, int index)
{
  return static_cast<bool>(value & (1 << index));
}
}  // namespace march4cpp
