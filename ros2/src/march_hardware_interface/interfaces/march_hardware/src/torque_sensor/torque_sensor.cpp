//
// Created by march on 10-1-23 by Marco Bak.
//

#include "../../include/march_hardware/torque_sensor/torque_sensor.h"

namespace march {
TorqueSensor::TorqueSensor(MotorControllerType motor_controller_type, float max_torque, float average_torque)
    : motor_controller_type_(motor_controller_type)
    , max_torque_(max_torque)
    , average_torque_(average_torque)
{
}

bool TorqueSensor::exceedsMaxTorque(float torque)
{
    return torque > max_torque_ || torque < -max_torque_;
}

MotorControllerType TorqueSensor::getMotorControllerType() const
{
    return motor_controller_type_;
}

float TorqueSensor::getMaxTorque() const
{
    return max_torque_;
}

float TorqueSensor::getAverageTorque() const
{
    return average_torque_;
}
} // namespace march
