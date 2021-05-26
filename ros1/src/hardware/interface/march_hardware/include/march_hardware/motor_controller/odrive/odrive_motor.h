#ifndef ODRIVE_INTERFACE_ODRIVE_MOTOR_H
#define ODRIVE_INTERFACE_ODRIVE_MOTOR_H

#include <utility>
#include <cmath>

#include "ros/ros.h"
#include "odrive.h"
#include "odrive_enums.h"
#include <march_hardware/motor_controller/actuation_mode.h>


static constexpr double MOTOR_KV = 95;
static constexpr double CURRENT_TO_TORQUE_CONVERSION = 8.27;

namespace march
{
class OdriveMotor : public Odrive
{
public:
  OdriveMotor(const std::string& axisNumber, std::shared_ptr<OdriveEndpoint> odriveEndpoint, ActuationMode mode,
              std::string json_config_file_path);
  ~OdriveMotor();

  bool initialize(int cycle_time) override;
  void prepareActuation() override;
  bool waitForIdleState(float timout = 30.0);

  void reset() override;

  void actuateRad(double target_rad) override;
  void actuateTorque(double target_torque_ampere) override;

  MotorControllerStates& getStates() override;

  float getMotorControllerVoltage() override;
  float getMotorCurrent() override;
  float getMotorVoltage() override;
  double getTorque() override;

  int32_t getAxisError();
  int32_t getAxisMotorError();
  int32_t getAxisEncoderError();
  int32_t getAxisControllerError();

  double getAngleRadAbsolute() override;
  double getVelocityRadAbsolute() override;

  double getAngleRadIncremental() override;
  double getVelocityRadIncremental() override;

  bool getIncrementalMorePrecise() const override;
  ActuationMode getActuationMode() const override
  {
    return this->mode_;
  }
  int getSlaveIndex() const override
  {
    return -1;
  }

private:
  int setState(int32_t state);
  int32_t getState();

  int getAngleCountsAbsolute();
  double getAngleCountsIncremental();

  ActuationMode mode_;
  std::string json_config_file_path_;

  void readValues();

  int32_t axis_error;
  int32_t axis_motor_error;
  int32_t axis_encoder_error;
  int32_t axis_controller_error;

  float motor_controller_voltage;
  float motor_current;
  float motor_voltage;

  double angle_counts_absolute;
  double velocity_rad_absolute;
  double angle_counts_incremental;
  double velocity_rad_incremental;
};

}  // namespace march
#endif  // ODRIVE_INTERFACE_ODRIVE_MOTOR_H
