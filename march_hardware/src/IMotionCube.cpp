// Copyright 2018 Project March.
#include "march_hardware/IMotionCube.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/error/motion_error.h"
#include "march_hardware/EtherCAT/EthercatSDO.h"
#include "march_hardware/EtherCAT/EthercatIO.h"

#include <iostream>
#include <iomanip>
#include <string>

#include <bitset>
#include <memory>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <utility>

#include <ros/ros.h>

namespace march
{
IMotionCube::IMotionCube(int slave_index, std::unique_ptr<AbsoluteEncoder> absolute_encoder,
                         std::unique_ptr<IncrementalEncoder> incremental_encoder, ActuationMode actuation_mode)
  : Slave(slave_index)
  , absolute_encoder_(std::move(absolute_encoder))
  , incremental_encoder_(std::move(incremental_encoder))
  , sw_string_("empty")
  , actuation_mode_(actuation_mode)
{
  if (!this->absolute_encoder_ || !this->incremental_encoder_)
  {
    throw std::invalid_argument("Incremental or absolute encoder cannot be nullptr");
  }
  this->absolute_encoder_->setSlaveIndex(slave_index);
  this->incremental_encoder_->setSlaveIndex(slave_index);
}
IMotionCube::IMotionCube(int slave_index, std::unique_ptr<AbsoluteEncoder> absolute_encoder,
                         std::unique_ptr<IncrementalEncoder> incremental_encoder, std::string& sw_stream,
                         ActuationMode actuation_mode)
  : IMotionCube(slave_index, std::move(absolute_encoder), std::move(incremental_encoder), actuation_mode)
{
  this->sw_string_ = std::move(sw_stream);
}

bool IMotionCube::writeInitialSDOs(int cycle_time)
{
  if (this->actuation_mode_ == ActuationMode::unknown)
  {
    throw error::HardwareException(error::ErrorType::INVALID_ACTUATION_MODE, "Cannot write initial settings to "
                                                                             "IMotionCube "
                                                                             "as it has actuation mode of unknown");
  }

  this->mapMisoPDOs();
  this->mapMosiPDOs();
  return this->writeInitialSettings(cycle_time);
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
  map_miso.addObject(IMCObjectName::MotorPosition);
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

// Set configuration parameters to the IMC
bool IMotionCube::writeInitialSettings(uint8_t cycle_time)
{
  int checksum_verified = verifySetup();

  if (!checksum_verified)
  {
    ROS_WARN("The .sw file for slave %d is not equal to the setup of the drive, downloading is necessary",
             this->slaveIndex);
    downloadSetupToDrive();
    checksum_verified = verifySetup();
    if (checksum_verified)
    {
      ROS_INFO("writing of the setup data has succeeded");
    }
    else
    {
      ROS_FATAL("writing of the setup data has failed");
    }
    return true;  // Resets all imcs and restart the EtherCAT train (necessary after downloading a "new" setup to the
                  // drive)
  }
  else
  {
    ROS_DEBUG("The .sw file for slave %d is equal to the setup of the drive.", this->slaveIndex);
  }

  // mode of operation
  int mode_of_op = sdo_bit8_write(slaveIndex, 0x6060, 0, this->actuation_mode_.toModeNumber());

  // position limit -- min position
  int min_pos_lim = sdo_bit32_write(slaveIndex, 0x607D, 1, this->absolute_encoder_->getLowerSoftLimitIU());

  // position limit -- max position
  int max_pos_lim = sdo_bit32_write(slaveIndex, 0x607D, 2, this->absolute_encoder_->getUpperSoftLimitIU());

  // Quick stop option
  int stop_opt = sdo_bit16_write(slaveIndex, 0x605A, 0, 6);

  // Quick stop deceleration
  int stop_decl = sdo_bit32_write(slaveIndex, 0x6085, 0, 0x7FFFFFFF);

  // Abort connection option code
  int abort_con = sdo_bit16_write(slaveIndex, 0x6007, 0, 1);

  // set the ethercat rate of encoder in form x*10^y
  int rate_ec_x = sdo_bit8_write(slaveIndex, 0x60C2, 1, cycle_time);
  int rate_ec_y = sdo_bit8_write(slaveIndex, 0x60C2, 2, -3);

  if (!(mode_of_op && max_pos_lim && min_pos_lim && stop_opt && stop_decl && abort_con && rate_ec_x && rate_ec_y &&
        checksum_verified))
  {
    throw error::HardwareException(error::ErrorType::WRITING_INITIAL_SETTINGS_FAILED,
                                   "Failed writing initial settings to IMC of slave %d", this->slaveIndex);
  }
  return false;
}

void IMotionCube::actuateRad(double target_rad)
{
  if (this->actuation_mode_ != ActuationMode::position)
  {
    throw error::HardwareException(error::ErrorType::INVALID_ACTUATION_MODE,
                                   "trying to actuate rad, while actuation mode is %s",
                                   this->actuation_mode_.toString().c_str());
  }

  if (std::abs(target_rad - this->getAngleRadAbsolute()) > MAX_TARGET_DIFFERENCE)
  {
    throw error::HardwareException(error::ErrorType::TARGET_EXCEEDS_MAX_DIFFERENCE,
                                   "Target %f exceeds max difference of %f from current %f for slave %d", target_rad,
                                   MAX_TARGET_DIFFERENCE, this->getAngleRadAbsolute(), this->slaveIndex);
  }
  this->actuateIU(this->absolute_encoder_->fromRad(target_rad));
}

void IMotionCube::actuateIU(int32_t target_iu)
{
  if (!this->absolute_encoder_->isValidTargetIU(this->getAngleIUAbsolute(), target_iu))
  {
    throw error::HardwareException(error::ErrorType::INVALID_ACTUATE_POSITION,
                                   "Position %d is invalid for slave %d. (%d, %d)", target_iu, this->slaveIndex,
                                   this->absolute_encoder_->getLowerSoftLimitIU(),
                                   this->absolute_encoder_->getUpperSoftLimitIU());
  }

  bit32 target_position = { .i = target_iu };

  uint8_t target_position_location = this->mosi_byte_offsets_.at(IMCObjectName::TargetPosition);

  set_output_bit32(this->slaveIndex, target_position_location, target_position);
}

void IMotionCube::actuateTorque(int16_t target_torque)
{
  if (this->actuation_mode_ != ActuationMode::torque)
  {
    throw error::HardwareException(error::ErrorType::INVALID_ACTUATION_MODE,
                                   "trying to actuate torque, while actuation mode is %s",
                                   this->actuation_mode_.toString().c_str());
  }

  if (target_torque >= MAX_TARGET_TORQUE)
  {
    throw error::HardwareException(error::ErrorType::TARGET_TORQUE_EXCEEDS_MAX_TORQUE,
                                   "Target torque of %d exceeds max torque of %d", target_torque, MAX_TARGET_TORQUE);
  }

  bit16 target_torque_struct = { .i = target_torque };

  uint8_t target_torque_location = this->mosi_byte_offsets_.at(IMCObjectName::TargetTorque);

  set_output_bit16(this->slaveIndex, target_torque_location, target_torque_struct);
}

double IMotionCube::getAngleRadAbsolute()
{
  if (!IMotionCubeTargetState::SWITCHED_ON.isReached(this->getStatusWord()) &&
      !IMotionCubeTargetState::OPERATION_ENABLED.isReached(this->getStatusWord()))
  {
    ROS_WARN_THROTTLE(10, "Invalid use of encoders, you're not in the correct state.");
  }
  return this->absolute_encoder_->getAngleRad(this->miso_byte_offsets_.at(IMCObjectName::ActualPosition));
}

double IMotionCube::getAngleRadIncremental()
{
  if (!IMotionCubeTargetState::SWITCHED_ON.isReached(this->getStatusWord()) &&
      !IMotionCubeTargetState::OPERATION_ENABLED.isReached(this->getStatusWord()))
  {
    ROS_WARN_THROTTLE(10, "Invalid use of encoders, you're not in the correct state.");
  }
  return this->incremental_encoder_->getAngleRad(this->miso_byte_offsets_.at(IMCObjectName::MotorPosition));
}

double IMotionCube::getAbsoluteRadPerBit() const
{
  return this->absolute_encoder_->getRadPerBit();
}

double IMotionCube::getIncrementalRadPerBit() const
{
  return this->incremental_encoder_->getRadPerBit();
}

int16_t IMotionCube::getTorque()
{
  bit16 return_byte = get_input_bit16(this->slaveIndex, this->miso_byte_offsets_.at(IMCObjectName::ActualTorque));
  return return_byte.i;
}

int32_t IMotionCube::getAngleIUAbsolute()
{
  return this->absolute_encoder_->getAngleIU(this->miso_byte_offsets_.at(IMCObjectName::ActualPosition));
}

int IMotionCube::getAngleIUIncremental()
{
  return this->incremental_encoder_->getAngleIU(this->miso_byte_offsets_.at(IMCObjectName::MotorPosition));
}

uint16_t IMotionCube::getStatusWord()
{
  return get_input_bit16(this->slaveIndex, this->miso_byte_offsets_.at(IMCObjectName::StatusWord)).ui;
}

uint16_t IMotionCube::getMotionError()
{
  return get_input_bit16(this->slaveIndex, this->miso_byte_offsets_.at(IMCObjectName::MotionErrorRegister)).ui;
}

uint16_t IMotionCube::getDetailedError()
{
  return get_input_bit16(this->slaveIndex, this->miso_byte_offsets_.at(IMCObjectName::DetailedErrorRegister)).ui;
}

float IMotionCube::getMotorCurrent()
{
  const float PEAK_CURRENT = 40.0;            // Peak current of iMC drive
  const float IU_CONVERSION_CONST = 65520.0;  // Conversion parameter, see Technosoft CoE programming manual

  int16_t motor_current_iu =
      get_input_bit16(this->slaveIndex, this->miso_byte_offsets_.at(IMCObjectName::ActualTorque)).i;
  return (2.0f * PEAK_CURRENT / IU_CONVERSION_CONST) *
         static_cast<float>(motor_current_iu);  // Conversion to Amp, see Technosoft CoE programming manual
}

float IMotionCube::getIMCVoltage()
{
  // maximum measurable DC voltage found in EMS Setup/Drive info button
  const float V_DC_MAX_MEASURABLE = 102.3;
  // Conversion parameter, see Technosoft CoE programming manual (2015 page 89)
  const float IU_CONVERSION_CONST = 65520.0;

  uint16_t imc_voltage_iu =
      get_input_bit16(this->slaveIndex, this->miso_byte_offsets_.at(IMCObjectName::DCLinkVoltage)).ui;
  return (V_DC_MAX_MEASURABLE / IU_CONVERSION_CONST) *
         static_cast<float>(imc_voltage_iu);  // Conversion to Volt, see Technosoft CoE programming manual
}

void IMotionCube::setControlWord(uint16_t control_word)
{
  bit16 control_word_ui = { .ui = control_word };
  set_output_bit16(slaveIndex, this->mosi_byte_offsets_.at(IMCObjectName::ControlWord), control_word_ui);
}

void IMotionCube::goToTargetState(IMotionCubeTargetState target_state)
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

void IMotionCube::goToOperationEnabled()
{
  if (this->actuation_mode_ == ActuationMode::unknown)
  {
    throw error::HardwareException(error::ErrorType::INVALID_ACTUATION_MODE, "Trying to go to operation enabled with "
                                                                             "unknown actuation mode");
  }
  this->setControlWord(128);

  this->goToTargetState(IMotionCubeTargetState::SWITCH_ON_DISABLED);
  this->goToTargetState(IMotionCubeTargetState::READY_TO_SWITCH_ON);
  this->goToTargetState(IMotionCubeTargetState::SWITCHED_ON);

  const int32_t angle = this->getAngleIUAbsolute();
  //  If the encoder is functioning correctly and the joint is not outside hardlimits, move the joint to its current
  //  position. Otherwise shutdown
  if (abs(angle) <= 2)
  {
    throw error::HardwareException(error::ErrorType::ENCODER_RESET,
                                   "Encoder of IMotionCube with slave index %d has reset. Read angle %d IU",
                                   this->slaveIndex, angle);
  }
  else if (!this->absolute_encoder_->isWithinHardLimitsIU(angle))
  {
    throw error::HardwareException(error::ErrorType::OUTSIDE_HARD_LIMITS,
                                   "Joint with slave index %d is outside hard limits (read value %d IU, limits from %d "
                                   "IU to %d IU)",
                                   this->slaveIndex, angle, this->absolute_encoder_->getLowerHardLimitIU(),
                                   this->absolute_encoder_->getUpperHardLimitIU());
  }

  if (this->actuation_mode_ == ActuationMode::position)
  {
    this->actuateIU(angle);
  }
  if (this->actuation_mode_ == ActuationMode::torque)
  {
    this->actuateTorque(0);
  }

  this->goToTargetState(IMotionCubeTargetState::OPERATION_ENABLED);
}

void IMotionCube::reset()
{
  this->setControlWord(0);
  ROS_DEBUG("Slave: %d, Try to reset IMC", this->slaveIndex);
  sdo_bit16_write(this->slaveIndex, 0x2080, 0, 1);
}

uint32_t IMotionCube::computeSWCheckSum(int& start_address, int& end_address)
{
  size_t pos = 0;
  size_t old_pos = 0;
  uint16_t sum = 0;
  std::string delimiter = "\n";
  std::string token;
  while ((pos = sw_string_.find(delimiter, old_pos)) != std::string::npos)
  {
    token = sw_string_.substr(old_pos, pos - old_pos);
    if (old_pos == 0)
    {
      start_address = std::stoi(token, nullptr, 16);
      token = "0";
    }
    if (pos - old_pos < 2)  // delimiter has length of 1 two \n in a row has difference in positions of 1
    {
      end_address = end_address + start_address - 2;  // The -2 compensates the offset of the end_address
      return sum;
    }
    end_address++;
    sum += std::stoi(token, nullptr, 16);  // State that we are looking at hexadecimal numbers
    old_pos = pos + 1;                     // Make sure to check the position after the previous one
  }
  throw error::HardwareException(error::ErrorType::INVALID_SW_STRING, "The .sw file has no delimiter of type %s",
                                 delimiter);
}

bool IMotionCube::verifySetup()
{
  int start_address = 0, end_address = 0;
  uint32_t sw_value = this->computeSWCheckSum(start_address, end_address);
  // set parameters to compute checksum on the imc
  int checksum_setup =
      sdo_bit32_write(slaveIndex, 0x2069, 0, end_address * 65536 + start_address);  // Endaddress leftshifted 4 times

  uint32_t imc_value;
  int value_size = sizeof(imc_value);
  // read computed checksum on imc
  int check_sum_read = sdo_bit32_read(slaveIndex, 0x206A, 0, value_size, imc_value);

  if (!(checksum_setup && check_sum_read))
  {
    throw error::HardwareException(error::ErrorType::WRITING_INITIAL_SETTINGS_FAILED,
                                   "Failed checking the checksum on slave: %d", this->slaveIndex);
  }

  ROS_DEBUG("The .sw checksum is : %d, and the drive checksum is %d", sw_value, imc_value);
  return sw_value == imc_value;
}

void IMotionCube::downloadSetupToDrive()
{
  int mem_location;
  int hex_ls_four = 65536;  // multiplying with this number will left-shift the other by 4 in hexadecimal notation
  uint32_t mem_setup = 9;   // send 16-bits and auto increment
  int result = 0;
  int final_result = 0;
  uint32_t data;

  size_t pos = 0;
  size_t old_pos = 0;
  std::string delimiter = "\n";
  std::string token, next_token;
  while ((pos = sw_string_.find(delimiter, old_pos)) != std::string::npos)
  {
    token = sw_string_.substr(old_pos, pos - old_pos);
    if (old_pos == 0)
    {
      mem_location = std::stoi(token, nullptr, 16) * hex_ls_four;
      result = sdo_bit32_write(slaveIndex, 0x2064, 0, mem_location + mem_setup);  // write the write-configuration
      final_result |= result;
    }
    else
    {
      if (pos - old_pos == delimiter.length())
      {
        break;
      }
      else
      {
        old_pos = pos + 1;
        pos = sw_string_.find(delimiter, old_pos);
        next_token = sw_string_.substr(old_pos, pos - old_pos);

        if (pos - old_pos != delimiter.length())
        {
          data = std::stoi(next_token, nullptr, 16) * hex_ls_four + std::stoi(token, nullptr, 16);
        }
        else
        {
          data = std::stoi(token, nullptr, 16);
        }
        result = sdo_bit32_write(slaveIndex, 0x2065, 0, data);  // write the write-configuration
      }
    }
    final_result &= result;
    old_pos = pos + 1;  // Make sure to check the position after the previous one
  }
  if (final_result == 0)
  {
    throw error::HardwareException(error::ErrorType::WRITING_INITIAL_SETTINGS_FAILED,
                                   "Failed writing .sw file to IMC of slave %d", this->slaveIndex);
  }
}

ActuationMode IMotionCube::getActuationMode() const
{
  return this->actuation_mode_;
}
}  // namespace march
