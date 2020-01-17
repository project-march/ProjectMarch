// Copyright 2018 Project March.
#include <march_hardware/IMotionCube.h>
#include <march_hardware/error/motion_error.h>
#include <march_hardware/EtherCAT/EthercatSDO.h>
#include <march_hardware/EtherCAT/EthercatIO.h>

#include <bitset>
#include <string>
#include <unistd.h>
#include <vector>

#include <ros/ros.h>

namespace march
{
IMotionCube::IMotionCube(int slave_index, Encoder encoder, ActuationMode actuation_mode)
  : Slave(slave_index), encoder(encoder), actuationMode(actuation_mode)
{
  this->encoder.setSlaveIndex(slave_index);
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
  this->misoByteOffsets = pdoMapMISO.map(this->slaveIndex, DataDirection::MISO);
}

// Map Process Data Object (PDO) for by sending SDOs to the IMC
// Master Out, Slave In
void IMotionCube::mapMosiPDOs()
{
  PDOmap pdoMapMOSI = PDOmap();
  pdoMapMOSI.addObject(IMCObjectName::ControlWord);  // Compulsory!
  pdoMapMOSI.addObject(IMCObjectName::TargetPosition);
  pdoMapMOSI.addObject(IMCObjectName::TargetTorque);
  this->mosiByteOffsets = pdoMapMOSI.map(this->slaveIndex, DataDirection::MOSI);
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

  if (this->actuationMode == ActuationMode::unknown)
  {
    throw std::runtime_error("Cannot write initial settings to IMotionCube as it has actuation mode of unknown");
  }

  // mode of operation
  int mode_of_op = sdo_bit8(slaveIndex, 0x6060, 0, this->actuationMode.toModeNumber());

  // position limit -- min position
  int max_pos_lim = sdo_bit32(slaveIndex, 0x607D, 1, this->encoder.getLowerSoftLimitIU());

  // position limit -- max position
  int min_pos_lim = sdo_bit32(slaveIndex, 0x607D, 2, this->encoder.getUpperSoftLimitIU());

  // Quick stop option
  int stop_opt = sdo_bit16(slaveIndex, 0x605A, 0, 6);

  // Quick stop deceleration
  int stop_decl = sdo_bit32(slaveIndex, 0x6085, 0, 0x7FFFFFFF);

  // Abort connection option code
  int abort_con = sdo_bit16(slaveIndex, 0x6007, 0, 1);

  // set the ethercat rate of encoder in form x*10^y
  int rate_ec_x = sdo_bit8(slaveIndex, 0x60C2, 1, ecatCycleTime);
  int rate_ec_y = sdo_bit8(slaveIndex, 0x60C2, 2, -3);

  if (!(mode_of_op && max_pos_lim && min_pos_lim && stop_opt && stop_decl && abort_con && rate_ec_x && rate_ec_y))
  {
    ROS_ERROR("Failed writing initial settings to IMC of slave %i", slaveIndex);
  }
}

void IMotionCube::actuateRad(float targetRad)
{
  ROS_ASSERT_MSG(this->actuationMode == ActuationMode::position, "trying to actuate rad, while actuationmode = %s",
                 this->actuationMode.toString().c_str());

  if (std::abs(targetRad - this->getAngleRad()) > 0.393)
  {
    ROS_ERROR("Target %f exceeds max difference of 0.393 from current %f for slave %d", targetRad, this->getAngleRad(),
              this->slaveIndex);
    throw std::runtime_error("Target exceeds max difference of 0.393 from current position");
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

  // The targetTorque must not exceed the value of 23500 IU, this is slightly larger than the current limit of the
  // linear joints defined in the urdf.
  ROS_ASSERT_MSG(targetTorque < 23500, "Torque of %d is too high.", targetTorque);

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
  const std::bitset<16> bitset(statusWord);
  if (bitset.test(0))
  {
    wordDescription += "Axis on. Power stage is enabled. Motor control is performed. ";
  }
  else
  {
    wordDescription += "Axis off. Power stage is disabled. Motor control is not performed. ";
  }
  if (bitset.test(2))
  {
    wordDescription += "Operation Enabled. ";
  }
  if (bitset.test(3))
  {
    wordDescription += "Fault. If set, a fault condition is or was present in the drive. ";
  }
  if (bitset.test(4))
  {
    wordDescription += "Motor supply voltage is present. ";
  }
  else
  {
    wordDescription += "Motor supply voltage is absent. ";
  }
  if (bitset.test(5))
  {
    wordDescription += "Quick Stop. When this bit is zero, the drive is performing a quick stop. ";
  }
  if (bitset.test(6))
  {
    wordDescription += "Switch On Disabled. ";
  }
  if (bitset.test(7))
  {
    wordDescription += "A TML function  was called, while another TML function is still in execution. ";
  }
  if (bitset.test(8))
  {
    wordDescription += "A TML function or homing is executed. ";
  }
  if (bitset.test(9))
  {
    wordDescription += "Remote - drive parameters may be modified via CAN. ";
  }
  else
  {
    wordDescription += "Remote - drive is in local mode and will not execute the command message. ";
  }
  if (bitset.test(10))
  {
    wordDescription += "Target reached. ";
  }
  if (bitset.test(11))
  {
    wordDescription += "Internal Limit Active. ";
  }
  if (bitset.test(12))
  {
    wordDescription += "Target position ignored. ";
  }
  if (bitset.test(13))
  {
    wordDescription += "Following error. ";
  }
  if (bitset.test(14))
  {
    wordDescription += "Last event set has occurred. ";
  }
  else
  {
    wordDescription += "No event set or the programmed event has not occurred yet. ";
  }
  if (bitset.test(15))
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

bool IMotionCube::goToTargetState(IMotionCubeTargetState targetState)
{
  ROS_DEBUG("\tTry to go to '%s'", targetState.getDescription().c_str());
  while (!targetState.isReached(this->getStatusWord()))
  {
    this->setControlWord(targetState.getControlWord());
    ROS_INFO_DELAYED_THROTTLE(5, "\tWaiting for '%s': %s", targetState.getDescription().c_str(),
                              std::bitset<16>(this->getStatusWord()).to_string().c_str());
    if (targetState.getState() == IMotionCubeTargetState::OPERATION_ENABLED.getState() &&
        this->getState(this->getStatusWord()) == IMCState::fault)
    {
      ROS_FATAL("IMotionCube went to fault state while attempting to go to '%s'. Shutting down.",
                targetState.getDescription().c_str());
      ROS_FATAL("Detailed Error: %s", error::parseDetailedError(this->getDetailedError()).c_str());
      ROS_FATAL("Motion Error: %s", error::parseMotionError(this->getMotionError()).c_str());
      throw std::domain_error("IMC to fault state");
    }
  }
  ROS_DEBUG("\tReached '%s'!", targetState.getDescription().c_str());
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
  //  If the encoder is functioning correctly and the joint is not outside hardlimits, move the joint to its current
  //  position. Otherwise shutdown
  if (abs(angleRead) <= 2)
  {
    ROS_FATAL("Encoder of IMotionCube with slaveIndex %d has reset. Read angle %d IU", this->slaveIndex, angleRead);
    throw std::domain_error("Encoder reset");
  }
  else if (!this->encoder.isWithinHardLimitsIU(angleRead))
  {
    ROS_FATAL("Joint with slaveIndex %d is outside hard limits (read value %d IU, limits from %d IU to %d IU)",
              this->slaveIndex, angleRead, this->encoder.getLowerHardLimitIU(), this->encoder.getUpperHardLimitIU());
    throw std::domain_error("Joint outside hard limits");
  }
  else
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
}  // namespace march
