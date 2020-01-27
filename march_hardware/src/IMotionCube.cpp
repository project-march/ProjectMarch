// Copyright 2018 Project March.
#include <march_hardware/IMotionCube.h>
#include <march_hardware/error/hardware_exception.h>
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
  : Slave(slave_index), encoder_(encoder), actuation_mode_(actuation_mode)
{
  this->encoder_.setSlaveIndex(slave_index);
}

void IMotionCube::writeInitialSDOs(int cycle_time)
{
  if (this->actuation_mode_ == ActuationMode::unknown)
  {
    throw error::HardwareException(error::ErrorType::INVALID_ACTUATION_MODE, "Cannot write initial settings to "
                                                                             "IMotionCube "
                                                                             "as it has actuation mode of unknown");
  }

  mapMisoPDOs();
  mapMosiPDOs();
  validateMisoPDOs();
  validateMosiPDOs();
  writeInitialSettings(cycle_time);
}

// Map Process Data Object (PDO) for by sending SDOs to the IMC
// Master In, Slave Out
void IMotionCube::mapMisoPDOs()
{
  PDOmap map_miso = PDOmap();
  map_miso.addObject(IMCObjectName::StatusWord);      // Compulsory!
  map_miso.addObject(IMCObjectName::ActualPosition);  // Compulsory!
  map_miso.addObject(IMCObjectName::ActualTorque);    // Compulsory!
  map_miso.addObject(IMCObjectName::MotionErrorRegister);
  map_miso.addObject(IMCObjectName::DetailedErrorRegister);
  map_miso.addObject(IMCObjectName::DCLinkVoltage);
  this->miso_byte_offsets_ = map_miso.map(this->slaveIndex, DataDirection::MISO);
}

// Map Process Data Object (PDO) for by sending SDOs to the IMC
// Master Out, Slave In
void IMotionCube::mapMosiPDOs()
{
  PDOmap map_mosi = PDOmap();
  map_mosi.addObject(IMCObjectName::ControlWord);  // Compulsory!
  map_mosi.addObject(IMCObjectName::TargetPosition);
  map_mosi.addObject(IMCObjectName::TargetTorque);
  this->mosi_byte_offsets_ = map_mosi.map(this->slaveIndex, DataDirection::MOSI);
}

// Checks if the compulsory MISO PDO objects are mapped
void IMotionCube::validateMisoPDOs()
{
  ROS_ASSERT_MSG(this->miso_byte_offsets_.count(IMCObjectName::StatusWord) == 1, "StatusWord not mapped");
  ROS_ASSERT_MSG(this->miso_byte_offsets_.count(IMCObjectName::ActualPosition) == 1, "ActualPosition not mapped");
  ROS_ASSERT_MSG(this->mosi_byte_offsets_.count(IMCObjectName::TargetTorque) == 1, "TargetTorque not mapped");

  if (this->actuation_mode_ == ActuationMode::position)
  {
    ROS_ASSERT_MSG(this->mosi_byte_offsets_.count(IMCObjectName::TargetPosition) == 1, "TargetPosition not mapped");
  }
}

// Checks if the compulsory MOSI PDO objects are mapped
void IMotionCube::validateMosiPDOs()
{
  ROS_ASSERT_MSG(this->mosi_byte_offsets_.count(IMCObjectName::ControlWord) == 1, "ControlWord not mapped");
}

// Set configuration parameters to the IMC
void IMotionCube::writeInitialSettings(uint8_t cycle_time)
{
  ROS_DEBUG("IMotionCube::writeInitialSettings");

  // mode of operation
  int mode_of_op = sdo_bit8(slaveIndex, 0x6060, 0, this->actuation_mode_.toModeNumber());

  // position limit -- min position
  int min_pos_lim = sdo_bit32(slaveIndex, 0x607D, 1, this->encoder_.getLowerSoftLimitIU());

  // position limit -- max position
  int max_pos_lim = sdo_bit32(slaveIndex, 0x607D, 2, this->encoder_.getUpperSoftLimitIU());

  // Quick stop option
  int stop_opt = sdo_bit16(slaveIndex, 0x605A, 0, 6);

  // Quick stop deceleration
  int stop_decl = sdo_bit32(slaveIndex, 0x6085, 0, 0x7FFFFFFF);

  // Abort connection option code
  int abort_con = sdo_bit16(slaveIndex, 0x6007, 0, 1);

  // set the ethercat rate of encoder in form x*10^y
  int rate_ec_x = sdo_bit8(slaveIndex, 0x60C2, 1, cycle_time);
  int rate_ec_y = sdo_bit8(slaveIndex, 0x60C2, 2, -3);

  if (!(mode_of_op && max_pos_lim && min_pos_lim && stop_opt && stop_decl && abort_con && rate_ec_x && rate_ec_y))
  {
    ROS_ERROR("Failed writing initial settings to IMC of slave %i", slaveIndex);
  }
}

void IMotionCube::actuateRad(float target_rad)
{
  if (this->actuation_mode_ != ActuationMode::position)
  {
    throw error::HardwareException(error::ErrorType::INVALID_ACTUATION_MODE,
                                   "trying to actuate rad, while actuation mode is %s",
                                   this->actuation_mode_.toString().c_str());
  }

  if (std::abs(target_rad - this->getAngleRad()) > 0.393)
  {
    ROS_ERROR("Target %f exceeds max difference of 0.393 from current %f for slave %d", target_rad, this->getAngleRad(),
              this->slaveIndex);
    throw std::runtime_error("Target exceeds max difference of 0.393 from current position");
  }
  this->actuateIU(this->encoder_.RadtoIU(target_rad));
}

void IMotionCube::actuateIU(int target_iu)
{
  if (!this->encoder_.isValidTargetIU(this->getAngleIU(), target_iu))
  {
    ROS_ERROR("Position %i is invalid for slave %d. (%d, %d)", target_iu, this->slaveIndex,
              this->encoder_.getLowerSoftLimitIU(), this->encoder_.getUpperSoftLimitIU());
    throw std::runtime_error("Invalid IU actuate command.");
  }

  bit32 target_position;
  target_position.i = target_iu;

  if (this->mosi_byte_offsets_.count(IMCObjectName::TargetPosition) != 1)
  {
    ROS_WARN("TargetPosition not defined in PDO mapping, so can't do actuateIU");
    return;
  }
  uint8_t target_position_location = this->mosi_byte_offsets_[IMCObjectName::TargetPosition];

  ROS_DEBUG("Trying to actuate slave %d, soem location %d to targetposition %d", this->slaveIndex,
            target_position_location, target_position.i);
  set_output_bit32(this->slaveIndex, target_position_location, target_position);
}

void IMotionCube::actuateTorque(int target_torque)
{
  if (this->actuation_mode_ != ActuationMode::torque)
  {
    throw error::HardwareException(error::ErrorType::INVALID_ACTUATION_MODE,
                                   "trying to actuate torque, while actuation mode is %s",
                                   this->actuation_mode_.toString().c_str());
  }

  // The targetTorque must not exceed the value of 23500 IU, this is slightly larger than the current limit of the
  // linear joints defined in the urdf.
  ROS_ASSERT_MSG(target_torque < 23500, "Torque of %d is too high.", target_torque);

  bit16 target_torque_struct;
  target_torque_struct.i = target_torque;

  if (this->mosi_byte_offsets_.count(IMCObjectName::TargetTorque) != 1)
  {
    ROS_WARN("TargetTorque not defined in PDO mapping, so can't do actuateTorque");
    return;
  }
  uint8_t target_torque_location = this->mosi_byte_offsets_[IMCObjectName::TargetTorque];

  ROS_DEBUG("Trying to actuate slave %d, soem location %d with target torque %d", this->slaveIndex,
            target_torque_location, target_torque_struct.i);
  set_output_bit16(this->slaveIndex, target_torque_location, target_torque_struct);
}

float IMotionCube::getAngleRad()
{
  ROS_ASSERT_MSG(this->miso_byte_offsets_.count(IMCObjectName::ActualPosition) == 1, "ActualPosition not defined in "
                                                                                     "PDO "
                                                                                     "mapping, so can't get angle");
  return this->encoder_.getAngleRad(this->miso_byte_offsets_[IMCObjectName::ActualPosition]);
}

float IMotionCube::getTorque()
{
  ROS_ASSERT_MSG(this->miso_byte_offsets_.count(IMCObjectName::ActualTorque) == 1, "ActualTorque not defined in PDO "
                                                                                   "mapping, so can't get torque");
  union bit16 return_byte = get_input_bit16(this->slaveIndex, this->miso_byte_offsets_[IMCObjectName::ActualTorque]);
  ROS_DEBUG("Actual Torque read: %d", return_byte.i);
  return return_byte.i;
}

int IMotionCube::getAngleIU()
{
  ROS_ASSERT_MSG(this->miso_byte_offsets_.count(IMCObjectName::ActualPosition) == 1, "ActualPosition not defined in "
                                                                                     "PDO "
                                                                                     "mapping, so can't get angle");
  return this->encoder_.getAngleIU(this->miso_byte_offsets_[IMCObjectName::ActualPosition]);
}

uint16_t IMotionCube::getStatusWord()
{
  ROS_ASSERT_MSG(this->miso_byte_offsets_.count(IMCObjectName::StatusWord) == 1, "StatusWord not defined in PDO "
                                                                                 "mapping, so can't get status word");
  return get_input_bit16(this->slaveIndex, this->miso_byte_offsets_[IMCObjectName::StatusWord]).ui;
}

uint16_t IMotionCube::getMotionError()
{
  if (this->miso_byte_offsets_.count(IMCObjectName::MotionErrorRegister) != 1)
  {
    ROS_WARN("MotionErrorRegister not defined in PDO mapping, so can't read it");
    return 0xFFFF;  // Not fatal, so can return
  }
  return get_input_bit16(this->slaveIndex, this->miso_byte_offsets_[IMCObjectName::MotionErrorRegister]).ui;
}

uint16_t IMotionCube::getDetailedError()
{
  if (this->miso_byte_offsets_.count(IMCObjectName::DetailedErrorRegister) != 1)
  {
    ROS_WARN("DetailedErrorRegister not defined in PDO mapping, so can't read it");
    return 0xFFFF;  // Not fatal, so can return
  }
  return get_input_bit16(this->slaveIndex, this->miso_byte_offsets_[IMCObjectName::DetailedErrorRegister]).ui;
}

float IMotionCube::getMotorCurrent()
{
  const float PEAK_CURRENT = 40.0;            // Peak current of iMC drive
  const float IU_CONVERSION_CONST = 65520.0;  // Conversion parameter, see Technosoft CoE programming manual
  if (this->miso_byte_offsets_.count(IMCObjectName::ActualTorque) != 1)
  {
    ROS_WARN("ActualTorque not defined in PDO mapping, so can't read it");
    return 0xFFFF;  // Not fatal, so can return
  }
  int16_t motor_current_iu = get_input_bit16(this->slaveIndex, this->miso_byte_offsets_[IMCObjectName::ActualTorque]).i;
  return (2.0f * PEAK_CURRENT / IU_CONVERSION_CONST) *
         static_cast<float>(motor_current_iu);  // Conversion to Amp, see Technosoft CoE programming manual
}

float IMotionCube::getMotorVoltage()
{
  const float V_DC_MAX_MEASURABLE = 102.3;    // maximum measurable DC voltage found in EMS Setup/Drive info button
  const float IU_CONVERSION_CONST = 65520.0;  // Conversion parameter, see Technosoft CoE programming manual
  if (this->miso_byte_offsets_.count(IMCObjectName::DCLinkVoltage) != 1)
  {
    ROS_WARN("DC-link Voltage not defined in PDO mapping, so can't read it");
    return 0xFFFF;  // Not fatal, so can return
  }
  uint16_t motor_voltage_iu =
      get_input_bit16(this->slaveIndex, this->miso_byte_offsets_[IMCObjectName::DCLinkVoltage]).ui;
  return (V_DC_MAX_MEASURABLE / IU_CONVERSION_CONST) *
         static_cast<float>(motor_voltage_iu);  // Conversion to Volt, see Technosoft CoE programming manual
}

void IMotionCube::setControlWord(uint16_t control_word)
{
  if (this->mosi_byte_offsets_.count(IMCObjectName::ControlWord) != 1)
  {
    ROS_FATAL("ControlWord not defined in PDO mapping, so can't set Control Word");
    throw std::exception();
  }
  bit16 controlwordu;
  controlwordu.i = control_word;
  set_output_bit16(slaveIndex, this->mosi_byte_offsets_[IMCObjectName::ControlWord], controlwordu);
}

std::string IMotionCube::parseStatusWord(uint16_t status_word)
{
  std::string description;
  const std::bitset<16> bitset(status_word);
  if (bitset.test(0))
  {
    description += "Axis on. Power stage is enabled. Motor control is performed. ";
  }
  else
  {
    description += "Axis off. Power stage is disabled. Motor control is not performed. ";
  }
  if (bitset.test(2))
  {
    description += "Operation Enabled. ";
  }
  if (bitset.test(3))
  {
    description += "Fault. If set, a fault condition is or was present in the drive. ";
  }
  if (bitset.test(4))
  {
    description += "Motor supply voltage is present. ";
  }
  else
  {
    description += "Motor supply voltage is absent. ";
  }
  if (bitset.test(5))
  {
    description += "Quick Stop. When this bit is zero, the drive is performing a quick stop. ";
  }
  if (bitset.test(6))
  {
    description += "Switch On Disabled. ";
  }
  if (bitset.test(7))
  {
    description += "A TML function  was called, while another TML function is still in execution. ";
  }
  if (bitset.test(8))
  {
    description += "A TML function or homing is executed. ";
  }
  if (bitset.test(9))
  {
    description += "Remote - drive parameters may be modified via CAN. ";
  }
  else
  {
    description += "Remote - drive is in local mode and will not execute the command message. ";
  }
  if (bitset.test(10))
  {
    description += "Target reached. ";
  }
  if (bitset.test(11))
  {
    description += "Internal Limit Active. ";
  }
  if (bitset.test(12))
  {
    description += "Target position ignored. ";
  }
  if (bitset.test(13))
  {
    description += "Following error. ";
  }
  if (bitset.test(14))
  {
    description += "Last event set has occurred. ";
  }
  else
  {
    description += "No event set or the programmed event has not occurred yet. ";
  }
  if (bitset.test(15))
  {
    description += "Axis on. Power stage is enabled. Motor control is performed. ";
  }
  else
  {
    description += "Axis off. Power stage is disabled. Motor control is not performed. ";
  }
}

bool IMotionCube::goToTargetState(IMotionCubeTargetState target_state)
{
  ROS_DEBUG("\tTry to go to '%s'", target_state.getDescription().c_str());
  while (!target_state.isReached(this->getStatusWord()))
  {
    this->setControlWord(target_state.getControlWord());
    ROS_INFO_DELAYED_THROTTLE(5, "\tWaiting for '%s': %s", target_state.getDescription().c_str(),
                              std::bitset<16>(this->getStatusWord()).to_string().c_str());
    if (target_state.getState() == IMotionCubeTargetState::OPERATION_ENABLED.getState() &&
        IMCState(this->getStatusWord()) == IMCState::FAULT)
    {
      ROS_FATAL("IMotionCube went to fault state while attempting to go to '%s'. Shutting down.",
                target_state.getDescription().c_str());
      ROS_FATAL("Detailed Error: %s", error::parseDetailedError(this->getDetailedError()).c_str());
      ROS_FATAL("Motion Error: %s", error::parseMotionError(this->getMotionError()).c_str());
      throw std::domain_error("IMC to fault state");
    }
  }
  ROS_DEBUG("\tReached '%s'!", target_state.getDescription().c_str());
}

bool IMotionCube::goToOperationEnabled()
{
  this->setControlWord(128);

  this->goToTargetState(IMotionCubeTargetState::SWITCH_ON_DISABLED);
  this->goToTargetState(IMotionCubeTargetState::READY_TO_SWITCH_ON);
  this->goToTargetState(IMotionCubeTargetState::SWITCHED_ON);
  // If ActualPosition is not defined in PDOmapping, a fatal error is thrown
  // because of safety reasons
  ROS_ASSERT_MSG(this->miso_byte_offsets_.count(IMCObjectName::ActualPosition) == 1, "ActualPosition not defined in "
                                                                                     "PDO "
                                                                                     "mapping, so can't get angle");

  int angle = this->encoder_.getAngleIU(this->miso_byte_offsets_[IMCObjectName::ActualPosition]);
  //  If the encoder is functioning correctly and the joint is not outside hardlimits, move the joint to its current
  //  position. Otherwise shutdown
  if (abs(angle) <= 2)
  {
    ROS_FATAL("Encoder of IMotionCube with slaveIndex %d has reset. Read angle %d IU", this->slaveIndex, angle);
    throw std::domain_error("Encoder reset");
  }
  else if (!this->encoder_.isWithinHardLimitsIU(angle))
  {
    ROS_FATAL("Joint with slaveIndex %d is outside hard limits (read value %d IU, limits from %d IU to %d IU)",
              this->slaveIndex, angle, this->encoder_.getLowerHardLimitIU(), this->encoder_.getUpperHardLimitIU());
    throw std::domain_error("Joint outside hard limits");
  }
  else
  {
    if (this->actuation_mode_ == ActuationMode::position)
    {
      this->actuateIU(angle);
    }
    if (this->actuation_mode_ == ActuationMode::torque)
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
  return this->actuation_mode_;
}
}  // namespace march
