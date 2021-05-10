// Copyright 2018 Project March.
#include "march_hardware/motor_controller/imotioncube/imotioncube.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/error/motor_controller_error.h"
#include "march_hardware/ethercat/pdo_types.h"
#include "march_hardware/motor_controller/actuation_mode.h"
#include "march_hardware/motor_controller/imotioncube/imotioncube_state.h"
#include "march_hardware/motor_controller/motor_controller.h"
#include <march_hardware/motor_controller/motor_controller_state.h>

#include <bitset>
#include <memory>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <utility>

#include <ros/ros.h>

namespace march {
IMotionCube::IMotionCube(const Slave& slave,
    std::unique_ptr<AbsoluteEncoder> absolute_encoder,
    std::unique_ptr<IncrementalEncoder> incremental_encoder,
    ActuationMode actuation_mode)
    : MotorController(slave, std::move(absolute_encoder),
        std::move(incremental_encoder), actuation_mode)
    , sw_string_(/*__s=*/"empty")
{
    if (!absolute_encoder_ || !incremental_encoder_) {
        throw error::HardwareException(error::ErrorType::MISSING_ENCODER,
            "An IMotionCube needs both an incremental and an absolute encoder");
    }
}

IMotionCube::IMotionCube(const Slave& slave,
    std::unique_ptr<AbsoluteEncoder> absolute_encoder,
    std::unique_ptr<IncrementalEncoder> incremental_encoder,
    std::string& sw_stream, ActuationMode actuation_mode)
    : IMotionCube(slave, std::move(absolute_encoder),
        std::move(incremental_encoder), actuation_mode)
{
    this->sw_string_ = std::move(sw_stream);
}

bool IMotionCube::initSdo(SdoSlaveInterface& sdo, int cycle_time)
{
    if (this->actuation_mode_ == ActuationMode::unknown) {
        throw error::HardwareException(error::ErrorType::INVALID_ACTUATION_MODE,
            "Cannot write initial settings to "
            "IMotionCube "
            "as it has actuation mode of unknown");
    }

    this->mapMisoPDOs(sdo);
    this->mapMosiPDOs(sdo);
    return this->writeInitialSettings(sdo, cycle_time);
}

// Map Process Data Object (PDO) for by sending SDOs to the IMC
// Master In, Slave Out
void IMotionCube::mapMisoPDOs(SdoSlaveInterface& sdo)
{
    IMCPDOmap map_miso;
    map_miso.addObject(IMCObjectName::StatusWord); // Compulsory!
    map_miso.addObject(IMCObjectName::ActualPosition); // Compulsory!
    map_miso.addObject(IMCObjectName::ActualTorque); // Compulsory!
    map_miso.addObject(IMCObjectName::MotionErrorRegister);
    map_miso.addObject(IMCObjectName::DetailedErrorRegister);
    map_miso.addObject(IMCObjectName::SecondDetailedErrorRegister);
    map_miso.addObject(IMCObjectName::DCLinkVoltage);
    map_miso.addObject(IMCObjectName::MotorVoltage);
    map_miso.addObject(IMCObjectName::MotorPosition);
    map_miso.addObject(IMCObjectName::MotorVelocity);
    map_miso.addObject(IMCObjectName::ActualVelocity);
    this->miso_byte_offsets_ = map_miso.map(sdo, DataDirection::MISO);
}

// Map Process Data Object (PDO) for by sending SDOs to the IMC
// Master Out, Slave In
void IMotionCube::mapMosiPDOs(SdoSlaveInterface& sdo)
{
    IMCPDOmap map_mosi;
    map_mosi.addObject(IMCObjectName::ControlWord); // Compulsory!
    map_mosi.addObject(IMCObjectName::TargetPosition);
    map_mosi.addObject(IMCObjectName::TargetTorque);
    this->mosi_byte_offsets_ = map_mosi.map(sdo, DataDirection::MOSI);
}

// Set configuration parameters to the IMC
bool IMotionCube::writeInitialSettings(SdoSlaveInterface& sdo, int cycle_time)
{
    bool checksum_verified = this->verifySetup(sdo);

    if (!checksum_verified) {
        ROS_WARN("The .sw file for slave %d is not equal to the setup of the "
                 "drive, downloading is necessary",
            this->getSlaveIndex());
        this->downloadSetupToDrive(sdo);
        checksum_verified = this->verifySetup(sdo);
        if (checksum_verified) {
            ROS_INFO("writing of the setup data has succeeded");
        } else {
            ROS_FATAL("writing of the setup data has failed");
        }
        return true; // Resets all imcs and restart the EtherCAT train
                     // (necessary after downloading a "new" setup to the drive)
    } else {
        ROS_DEBUG(
            "The .sw file for slave %d is equal to the setup of the drive.",
            this->getSlaveIndex());
    }

    /* All addresses were retrieved from the IMC Manual:
    https://technosoftmotion.com/wp-content/uploads/P091.025.iMOTIONCUBE.CAN_.CAT_.UM_-1.pdf
  */
    // mode of operation
    int mode_of_op = sdo.write(
        /*index=*/0x6060, /*sub=*/0, getActuationModeNumber());

    // position limit -- min position
    int min_pos_lim = sdo.write(/*index=*/0x607D, /*sub=*/1,
        this->getAbsoluteEncoder()->getLowerSoftLimitIU());

    // position limit -- max position
    int max_pos_lim = sdo.write(/*index=*/0x607D, /*sub=*/2,
        this->getAbsoluteEncoder()->getUpperSoftLimitIU());

    // Quick stop option
    int stop_opt = sdo.write<int16_t>(/*index=*/0x605A, /*sub=*/0, /*value=*/6);

    // Quick stop deceleration
    int stop_decl = sdo.write<uint32_t>(
        /*index=*/0x6085, /*sub=*/0, /*value=*/0x7FFFFFFF);

    // Abort connection option code
    int abort_con
        = sdo.write<int16_t>(/*index=*/0x6007, /*sub=*/0, /*value=*/1);

    // set the ethercat rate of encoder in form x*10^y
    int rate_ec_x = sdo.write<uint8_t>(/*index=*/0x60C2, /*sub=*/1, cycle_time);
    int rate_ec_y
        = sdo.write<int8_t>(/*index=*/0x60C2, /*sub=*/2, /*value=*/-3);

    // use filter object to read motor voltage
    int volt_address
        = sdo.write<int16_t>(/*index=*/0x2108, /*sub=*/1, /*value=*/0x0232);
    int volt_filter
        = sdo.write<int16_t>(/*index=*/0x2108, /*sub=*/2, /*value=*/32767);

    if (!(mode_of_op && max_pos_lim && min_pos_lim && stop_opt && stop_decl
            && abort_con && rate_ec_x && rate_ec_y && volt_address
            && volt_filter)) {
        throw error::HardwareException(
            error::ErrorType::WRITING_INITIAL_SETTINGS_FAILED,
            "Failed writing initial settings to IMC of slave %d",
            this->getSlaveIndex());
    }
    return false;
}

void IMotionCube::actuateRadians(float target_position)
{
    if (this->actuation_mode_ != ActuationMode::position) {
        throw error::HardwareException(error::ErrorType::INVALID_ACTUATION_MODE,
            "trying to actuate rad, while actuation mode is %s",
            this->actuation_mode_.toString().c_str());
    }

    if (std::abs(target_position - this->getAbsolutePositionUnchecked())
        > MAX_TARGET_DIFFERENCE) {
        throw error::HardwareException(
            error::ErrorType::TARGET_EXCEEDS_MAX_DIFFERENCE,
            "Target %f exceeds max difference of %f from current %f for slave "
            "%d",
            target_position, MAX_TARGET_DIFFERENCE,
            this->getAbsolutePositionUnchecked(), this->getSlaveIndex());
    }
    this->actuateIU(this->absolute_encoder_->toIU(
        target_position, /*use_zero_position=*/true));
}

void IMotionCube::actuateIU(int32_t target_iu)
{
    if (!this->getAbsoluteEncoder()->isValidTargetIU(
            this->getAbsolutePositionIU(), target_iu)) {
        throw error::HardwareException(
            error::ErrorType::INVALID_ACTUATE_POSITION,
            "Position %d is invalid for slave %d. (%d, %d)", target_iu,
            this->getSlaveIndex(),
            this->absolute_encoder_->getLowerSoftLimitIU(),
            this->absolute_encoder_->getUpperSoftLimitIU());
    }

    bit32 target_position = { .i = target_iu };

    uint8_t target_position_location
        = this->mosi_byte_offsets_.at(IMCObjectName::TargetPosition);

    this->write32(target_position_location, target_position);
}

void IMotionCube::actuateTorque(float target_torque)
{
    auto target_torque_iu = (int16_t)std::round(target_torque);
    if (this->actuation_mode_ != ActuationMode::torque) {
        throw error::HardwareException(error::ErrorType::INVALID_ACTUATION_MODE,
            "trying to actuate torque, while actuation mode is %s",
            this->actuation_mode_.toString().c_str());
    }

    if (target_torque >= MAX_TARGET_TORQUE) {
        throw error::HardwareException(
            error::ErrorType::TARGET_TORQUE_EXCEEDS_MAX_TORQUE,
            "Target torque of %d exceeds max torque of %d", target_torque,
            MAX_TARGET_TORQUE);
    }

    bit16 target_torque_struct = { .i = target_torque_iu };

    uint8_t target_torque_location
        = this->mosi_byte_offsets_.at(IMCObjectName::TargetTorque);

    this->write16(target_torque_location, target_torque_struct);
}

float IMotionCube::getTorque()
{
    bit16 return_byte = this->read16(
        this->miso_byte_offsets_.at(IMCObjectName::ActualTorque));
    return return_byte.i;
}

int32_t IMotionCube::getAbsolutePositionIU()
{
    if (!IMotionCubeTargetState::SWITCHED_ON.isReached(this->getStatusWord())
        && !IMotionCubeTargetState::OPERATION_ENABLED.isReached(
            this->getStatusWord())) {
        ROS_WARN_THROTTLE(
            10, "Invalid use of encoders, you're not in the correct state.");
    }
    bit32 return_byte = this->read32(
        this->miso_byte_offsets_.at(IMCObjectName::ActualPosition));
    return return_byte.i;
}

int32_t IMotionCube::getIncrementalPositionIU()
{
    if (!IMotionCubeTargetState::SWITCHED_ON.isReached(this->getStatusWord())
        && !IMotionCubeTargetState::OPERATION_ENABLED.isReached(
            this->getStatusWord())) {
        ROS_WARN_THROTTLE(
            10, "Invalid use of encoders, you're not in the correct state.");
    }
    bit32 return_byte = this->read32(
        this->miso_byte_offsets_.at(IMCObjectName::MotorPosition));
    return return_byte.i;
}

float IMotionCube::getAbsoluteVelocityIU()
{
    bit32 return_byte = this->read32(
        this->miso_byte_offsets_.at(IMCObjectName::ActualVelocity));
    return return_byte.i
        / (TIME_PER_VELOCITY_SAMPLE * FIXED_POINT_TO_FLOAT_CONVERSION);
}

float IMotionCube::getIncrementalVelocityIU()
{
    bit32 return_byte = this->read32(
        this->miso_byte_offsets_.at(IMCObjectName::MotorVelocity));
    return return_byte.i
        / (TIME_PER_VELOCITY_SAMPLE * FIXED_POINT_TO_FLOAT_CONVERSION);
}

uint16_t IMotionCube::getStatusWord()
{
    return this->read16(this->miso_byte_offsets_.at(IMCObjectName::StatusWord))
        .ui;
}

uint16_t IMotionCube::getMotionError()
{
    return this
        ->read16(
            this->miso_byte_offsets_.at(IMCObjectName::MotionErrorRegister))
        .ui;
}

uint16_t IMotionCube::getDetailedError()
{
    return this
        ->read16(
            this->miso_byte_offsets_.at(IMCObjectName::DetailedErrorRegister))
        .ui;
}

uint16_t IMotionCube::getSecondDetailedError()
{
    return this
        ->read16(this->miso_byte_offsets_.at(
            IMCObjectName::SecondDetailedErrorRegister))
        .ui;
}

float IMotionCube::getMotorCurrent()
{
    const float PEAK_CURRENT = 40.0; // Peak current of iMC drive
    const float IU_CONVERSION_CONST
        = 65520.0; // Conversion parameter, see Technosoft CoE programming
                   // manual

    int16_t motor_current_iu
        = this->read16(this->miso_byte_offsets_.at(IMCObjectName::ActualTorque))
              .i;
    return (2.0F * PEAK_CURRENT / IU_CONVERSION_CONST)
        * static_cast<float>(
            motor_current_iu); // Conversion to Amp, see Technosoft CoE
    // programming manual
}

float IMotionCube::getMotorControllerVoltage()
{
    // maximum measurable DC voltage found in EMS Setup/Drive info button
    const float V_DC_MAX_MEASURABLE = 102.3;
    // Conversion parameter, see Technosoft CoE programming manual (2015 page
    // 89)
    const float IU_CONVERSION_CONST = 65520.0;

    uint16_t imc_voltage_iu = this->read16(this->miso_byte_offsets_.at(
                                               IMCObjectName::DCLinkVoltage))
                                  .ui;
    return (V_DC_MAX_MEASURABLE / IU_CONVERSION_CONST)
        * static_cast<float>(
            imc_voltage_iu); // Conversion to Volt, see Technosoft CoE
                             // programming manual
}

float IMotionCube::getMotorVoltage()
{
    return this->read16(
                   this->miso_byte_offsets_.at(IMCObjectName::MotorVoltage))
        .ui;
}

void IMotionCube::setControlWord(uint16_t control_word)
{
    bit16 control_word_ui = { .ui = control_word };
    this->write16(this->mosi_byte_offsets_.at(IMCObjectName::ControlWord),
        control_word_ui);
}

void IMotionCube::goToTargetState(const IMotionCubeTargetState& target_state)
{
    ROS_DEBUG("\tTry to go to '%s'", target_state.getDescription().c_str());
    while (!target_state.isReached(this->getStatusWord())) {
        this->setControlWord(target_state.getControlWord());
        ROS_INFO_DELAYED_THROTTLE(5, "\tWaiting for '%s': %s",
            target_state.getDescription().c_str(),
            std::bitset<16>(this->getStatusWord()).to_string().c_str());
        if (target_state.getState()
                == IMotionCubeTargetState::OPERATION_ENABLED.getState()
            && IMCStateOfOperation(this->getStatusWord())
                == IMCStateOfOperation::FAULT) {
            ROS_FATAL("IMotionCube went to fault state while attempting to go "
                      "to '%s'. Shutting down.",
                target_state.getDescription().c_str());
            ROS_FATAL("Motion Error (MER): %s",
                error::parseError(this->getMotionError(),
                    error::ErrorRegister::IMOTIONCUBE_MOTION_ERROR)
                    .c_str());
            ROS_FATAL("Detailed Error (DER): %s",
                error::parseError(this->getDetailedError(),
                    error::ErrorRegister::IMOTIONCUBE_DETAILED_MOTION_ERROR)
                    .c_str());
            ROS_FATAL("Detailed Error 2 (DER2): %s",
                error::parseError(this->getSecondDetailedError(),
                    error::ErrorRegister::
                        IMOTIONCUBE_SECOND_DETAILED_MOTION_ERROR)
                    .c_str());

            throw std::domain_error("IMC to fault state");
        }
    }
    ROS_DEBUG("\tReached '%s'!", target_state.getDescription().c_str());
}

void IMotionCube::prepareActuation()
{
    if (this->actuation_mode_ == ActuationMode::unknown) {
        throw error::HardwareException(error::ErrorType::INVALID_ACTUATION_MODE,
            "Trying to go to operation enabled with "
            "unknown actuation mode");
    }
    this->setControlWord(/*control_word=*/128);

    this->goToTargetState(IMotionCubeTargetState::SWITCH_ON_DISABLED);
    this->goToTargetState(IMotionCubeTargetState::READY_TO_SWITCH_ON);
    this->goToTargetState(IMotionCubeTargetState::SWITCHED_ON);

    const int32_t angle = this->getAbsolutePositionIU();
    //  If the encoder is functioning correctly and the joint is not outside
    //  hardlimits, move the joint to its current position. Otherwise shutdown
    if (abs(angle) <= 2) {
        throw error::HardwareException(error::ErrorType::ENCODER_RESET,
            "Encoder of IMotionCube with slave index %d has reset. Read angle "
            "%d IU",
            this->getSlaveIndex(), angle);
    } else if (!this->getAbsoluteEncoder()->isWithinHardLimitsIU(angle)) {
        throw error::HardwareException(error::ErrorType::OUTSIDE_HARD_LIMITS,
            "Joint with slave index %d is outside hard limits (read value %d "
            "IU, limits from %d "
            "IU to %d IU)",
            this->getSlaveIndex(), angle,
            this->getAbsoluteEncoder()->getLowerHardLimitIU(),
            this->getAbsoluteEncoder()->getUpperHardLimitIU());
    }

    if (this->actuation_mode_ == ActuationMode::position) {
        this->actuateIU(angle);
    }
    if (this->actuation_mode_ == ActuationMode::torque) {
        this->actuateTorque(/*target_torque=*/0);
    }

    this->goToTargetState(IMotionCubeTargetState::OPERATION_ENABLED);
}

void IMotionCube::reset(SdoSlaveInterface& sdo)
{
    this->setControlWord(/*control_word=*/0);
    ROS_DEBUG("Slave: %d, Try to reset IMC", this->getSlaveIndex());
    sdo.write<uint16_t>(/*index=*/0x2080, /*sub=*/0, /*value=*/1);
}

uint16_t IMotionCube::computeSWCheckSum(
    uint16_t& start_address, uint16_t& end_address)
{
    size_t pos = 0;
    size_t old_pos = 0;
    uint16_t sum = 0;
    const std::string delimiter = "\n";
    while ((pos = sw_string_.find(delimiter, old_pos)) != std::string::npos) {
        std::string token = sw_string_.substr(old_pos, pos - old_pos);
        if (old_pos == 0) {
            start_address = std::stoi(token, /*__idx=*/nullptr, /*__base=*/16);
            token = "0";
        }
        if (pos == old_pos) {
            end_address = end_address + start_address
                - 1; // The -1 compensates the offset of the end_address
            return sum;
        }
        end_address++;
        sum += std::stoi(token, /*__idx=*/nullptr,
            /*__base=*/16); // State that we are looking at hexadecimal numbers
        old_pos = pos + delimiter.length(); // Make sure to check the position
                                            // after the previous one
    }
    throw error::HardwareException(error::ErrorType::INVALID_SW_STRING,
        "The .sw file has no delimiter of type %s", delimiter.c_str());
}

bool IMotionCube::verifySetup(SdoSlaveInterface& sdo)
{
    uint16_t start_address = 0;
    uint16_t end_address = 0;
    const uint32_t sw_value
        = this->computeSWCheckSum(start_address, end_address);
    // set parameters to compute checksum on the imc
    const int checksum_setup = sdo.write<uint32_t>(
        /*index=*/0x2069, /*sub=*/0,
        (uint32_t)(end_address << 16U) | start_address);

    uint16_t imc_value;
    int value_size = sizeof(imc_value);
    // read computed checksum on imc
    const int check_sum_read = sdo.read<uint16_t>(
        /*index=*/0x206A, /*sub=*/0, value_size, imc_value);
    if (!(checksum_setup && check_sum_read)) {
        throw error::HardwareException(
            error::ErrorType::WRITING_INITIAL_SETTINGS_FAILED,
            "Failed checking the checksum on slave: %d", this->getSlaveIndex());
    }

    ROS_DEBUG("The .sw checksum is : %d, and the drive checksum is %d",
        sw_value, imc_value);
    return sw_value == imc_value;
}

void IMotionCube::downloadSetupToDrive(SdoSlaveInterface& sdo)
{
    uint32_t result = 0;
    uint32_t final_result = 0;

    size_t pos = 0;
    size_t old_pos = 0;
    const std::string delimiter = "\n";
    while ((pos = sw_string_.find(delimiter, old_pos)) != std::string::npos) {
        const std::string token = sw_string_.substr(old_pos, pos - old_pos);
        if (old_pos == 0) {
            const uint16_t mem_location
                = std::stoi(token, /*__idx=*/nullptr, /*__base=*/16);
            const uint16_t mem_setup = 9; // send 16-bits and auto increment
            result = (uint32_t)sdo.write<uint32_t>(/*index=*/0x2064, /*sub=*/0,
                (uint32_t)(mem_location << 16U)
                    | mem_setup); // write the write-configuration
            final_result |= result;
        } else {
            if (pos == old_pos) {
                break;
            } else {
                old_pos = pos + delimiter.length();
                pos = sw_string_.find(delimiter, old_pos);
                const std::string next_token
                    = sw_string_.substr(old_pos, pos - old_pos);

                uint32_t data = 0;
                if (pos != old_pos) {
                    data = ((uint32_t)std::stoi(
                                next_token, /*__idx=*/nullptr, /*__base=*/16)
                               << 16U)
                        | (uint32_t)std::stoi(
                            token, /*__idx=*/nullptr, /*__base=*/16);
                } else {
                    data = (uint32_t)std::stoi(
                        token, /*__idx=*/nullptr, /*__base=*/16);
                }
                result = (uint32_t)sdo.write<uint32_t>(
                    /*index=*/0x2065, /*sub=*/0,
                    data); // write the write-configuration
            }
        }
        final_result &= result;
        old_pos = pos + delimiter.length(); // Make sure to check the position
                                            // after the previous one
    }
    if (final_result == 0) {
        throw error::HardwareException(
            error::ErrorType::WRITING_INITIAL_SETTINGS_FAILED,
            "Failed writing .sw file to IMC of slave %d",
            this->getSlaveIndex());
    }
}

int IMotionCube::getActuationModeNumber() const
{
    /* These values were retrieved from the IMC manual, section 4.2.4 */
    switch (this->actuation_mode_.getValue()) {
        case ActuationMode::position:
            return 8;
        case ActuationMode::torque:
            return 10;
        default:
            return 0;
    }
}

std::unique_ptr<MotorControllerState> IMotionCube::getState()
{
    auto state = std::make_unique<IMotionCubeState>();

    state->state_of_operation_ = IMCStateOfOperation(getStatusWord());

    state->motion_error_ = getMotionError();
    state->detailed_error_ = getDetailedError();
    state->second_detailed_error_ = getSecondDetailedError();

    state->motor_current_ = getMotorCurrent();
    state->motor_controller_voltage_ = getMotorControllerVoltage();
    state->motor_voltage_ = getMotorVoltage();

    state->absolute_position_iu_ = getAbsolutePositionIU();
    state->incremental_position_iu_ = getIncrementalPositionIU();
    state->absolute_velocity_iu_ = getAbsoluteVelocityIU();
    state->incremental_velocity_iu_ = getIncrementalVelocityIU();

    state->absolute_position_ = getAbsolutePositionUnchecked();
    state->incremental_position_ = getIncrementalPosition();
    state->absolute_velocity_ = getAbsoluteVelocityUnchecked();
    state->incremental_velocity_ = getIncrementalVelocity();

    return state;
}

float IMotionCube::getAbsolutePositionUnchecked()
{
    return (float)this->getAbsoluteEncoder()->toRadians(
        getAbsolutePositionIU(), /*use_zero_position=*/true);
}

float IMotionCube::getIncrementalPositionUnchecked()
{
    return (float)this->getIncrementalEncoder()->toRadians(
        getIncrementalPositionIU(), true);
}

float IMotionCube::getAbsoluteVelocityUnchecked()
{
    return (float)this->getAbsoluteEncoder()->toRadians(
        getAbsoluteVelocityIU(), /*use_zero_position=*/false);
}

float IMotionCube::getIncrementalVelocityUnchecked()
{
    return (float)this->getIncrementalEncoder()->toRadians(
        getIncrementalVelocityIU(), false);
}

} // namespace march
