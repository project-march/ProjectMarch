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
#include "march_hardware/torque_sensor/torque_sensor.h"

#include <memory>
#include <string>
#include <unordered_map>

namespace march {
class ODrive : public MotorController {
public:
    /**
     * Constructs an ODrive with an incremental and absolute encoder.
     *
     * @param slave slave of the ODrive.
     * @param absolute_encoder pointer to absolute encoder, required so cannot be nullptr.
     * @param incremental_encoder pointer to incremental encoder, required so cannot be nullptr.
     * @param torque_sensor pointer to the torque sensor, required so cannot be nullptr.
     * @param actuation_mode actuation mode in which the ODrive must operate.
     * @param use_inc_enc_for_position whether to use the incremental encoder for position updates.
     * @param logger The logger to print warning or info to the terminal.
     * @throws error::HardwareException When an absolute encoder is nullptr.
     */
    ODrive(const Slave& slave, ODriveAxis axis, std::unique_ptr<AbsoluteEncoder> absolute_encoder,
        std::unique_ptr<IncrementalEncoder> incremental_encoder, std::unique_ptr<TorqueSensor> torque_sensor,
        ActuationMode actuation_mode, bool use_inc_enc_for_position,bool index_found,std::shared_ptr<march_logger::BaseLogger> logger);

    ~ODrive() noexcept override = default;

    // Override functions for actuating the ODrive
    std::chrono::nanoseconds prepareActuation() override;
    void enableActuation() override;
    void actuateTorque(float target_torque, float fuzzy_weight) override;
    void actuateRadians(float target_position, float fuzzy_weight) override;
    void sendTargetPosition(float target_position) override;

     void sendPID(std::array<double, 3> pos_pid, std::array<double, 2> tor_pid) override;

    // Override reset function
    std::chrono::nanoseconds reset() override;

    bool requiresUniqueSlaves() const override;

    // Transform the ActuationMode to a number that is understood by the ODrive
    int getActuationModeNumber() const override;

    // Get a full description of the state of the ODrive
    std::unique_ptr<MotorControllerState> getState() override;

    // Getters for specific information about the state of the motor and the
    // ODrive
    //    float getTorque() override;
    float getMotorCurrent() override;
    float getMotorControllerVoltage() override;
    float getMotorVoltage() override;
    float getActualEffort() override;
    float getMotorTemperature();
    float getOdriveTemperature();

    double getTorqueLimit() const override;
    static constexpr double TORQUE_LIMIT = 100.0; // TODO: Determine a better value here

protected:
    // Override protected functions from MotorController class
    float getAbsolutePositionUnchecked() override;
    float getIncrementalPositionUnchecked() override;
    float getAbsoluteVelocityUnchecked() override;
    float getIncrementalVelocityUnchecked() override;
    float getTorqueUnchecked() override;

private:
    // Getter and setter for the axis state
    void setAxisState(ODriveAxisState state);
    void waitForState(ODriveAxisState target_state);
    ODriveAxisState getAxisState();

    uint32_t getAbsolutePositionIU();
    int32_t getIncrementalPositionIU();
    float getIncrementalVelocityIU();
    float getAIEAbsolutePositionRad(); 
    uint32_t getCheckSum();

    uint32_t getODriveError();
    uint32_t getAxisError();
    uint64_t getMotorError();
    uint32_t getEncoderError();
    uint32_t getTorqueSensorError();
    uint32_t getControllerError();

    ODriveAxis axis_;
    bool index_found_;
};

} // namespace march

#endif // MARCH_HARDWARE_ODRIVE_H
