// Copyright 2021 Project March.

#ifndef MARCH_HARDWARE_ODRIVE_H
#define MARCH_HARDWARE_ODRIVE_H

#include "march_hardware/encoder/absolute_encoder.h"
#include "march_hardware/encoder/incremental_encoder.h"
#include "march_hardware/ethercat/odrive_pdo_map.h"
#include "march_hardware/ethercat/pdo_types.h"
#include "march_hardware/ethercat/sdo_interface.h"
#include "march_hardware/ethercat/slave.h"
#include "march_hardware/motor_controller/actuation_mode.h"
#include "march_hardware/motor_controller/motor_controller.h"
#include "march_hardware/motor_controller/odrive/odrive_state.h"

#include <memory>
#include <string>
#include <unordered_map>

/* For more info see
 * https://docs.odriverobotics.com/
 * https://discourse.odriverobotics.com/t/where-does-the-formula-for-calculating-torque-come-from/1169
 */
#define KV_TO_TORQUE_CONSTANT 8.27

namespace march {
class ODrive : public MotorController {
public:
    /**
     * Constructs an ODrive with an incremental and absolute encoder.
     *
     * @param slave slave of the ODrive
     * @param absolute_encoder pointer to absolute encoder, required so cannot
     * be nullptr
     * @param incremental_encoder pointer to incremental encoder, required so
     * cannot be nullptr
     * @param actuation_mode actuation mode in which the ODrive must
     * operate
     * @throws error::HardwareException When an absolute encoder is nullptr.
     */
    ODrive(const Slave& slave, ODriveAxis axis,
        std::unique_ptr<AbsoluteEncoder> absolute_encoder,
        std::unique_ptr<IncrementalEncoder> incremental_encoder,
        ActuationMode actuation_mode, bool pre_calibrated,
        unsigned int motor_kv);
    ODrive(const Slave& slave, ODriveAxis axis,
        std::unique_ptr<AbsoluteEncoder> absolute_encoder,
        ActuationMode actuation_mode, bool pre_calibrated,
        unsigned int motor_kv);
    ODrive(const Slave& slave, ODriveAxis axis,
        std::unique_ptr<AbsoluteEncoder> absolute_encoder,
        ActuationMode actuation_mode, unsigned int motor_kv);

    ~ODrive() noexcept override = default;

    // Override functions for actuating the ODrive
    void prepareActuation() override;
    void actuateTorque(float target_torque) override;
    void actuateRadians(float target_position) override;
    bool isIncrementalEncoderMorePrecise() const override;

    bool hasUniqueSlaves() const override;

    // Transform the ActuationMode to a number that is understood by the ODrive
    int getActuationModeNumber() const override;

    // Get a full description of the state of the ODrive
    std::unique_ptr<MotorControllerState> getState() override;

    // Getters for specific information about the state of the motor and the
    // ODrive
    float getTorque() override;
    float getMotorCurrent() override;
    float getMotorControllerVoltage() override;
    float getMotorVoltage() override;
    float getActualEffort() override;

protected:
    // Override protected functions from Slave class
    bool initSdo(SdoSlaveInterface& sdo, int cycle_time) override;
    void reset(SdoSlaveInterface& sdo) override;

    // Override protected functions from MotorController class
    float getAbsolutePositionUnchecked() override;
    float getIncrementalPositionUnchecked() override;
    float getAbsoluteVelocityUnchecked() override;
    float getIncrementalVelocityUnchecked() override;

private:
    // Getter and setter for the axis state
    void setAxisState(ODriveAxisState state);
    void waitForState(ODriveAxisState target_state);
    ODriveAxisState getAxisState();

    int32_t getAbsolutePositionIU();
    int32_t getIncrementalPositionIU();
    float getIncrementalVelocityIU();

    uint32_t getAxisError();
    uint32_t getMotorError();
    uint32_t getEncoderManagerError();
    uint32_t getEncoderError();
    uint32_t getControllerError();

    // Get the direction of the most significant encoder
    Encoder::Direction getMotorDirection() const;

    ODriveAxis axis_;
    bool pre_calibrated_;
    float torque_constant_;
};

} // namespace march

#endif // MARCH_HARDWARE_ODRIVE_H
