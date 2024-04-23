//
// Created by march on 10-1-23 by Marco Bak.
//

#ifndef BUILD_TORQUE_SENSOR_HPP
#define BUILD_TORQUE_SENSOR_HPP
#include "march_hardware/motor_controller/motor_controller_type.h"

#include <march_logger_cpp/base_logger.hpp>
#include <ostream>

namespace march {
class TorqueSensor {
public:
    TorqueSensor(MotorControllerType motor_controller_type_, float max_torque_, float average_torque_);

    ~TorqueSensor() noexcept = default;

    bool exceedsMaxTorque(float torque);

    MotorControllerType getMotorControllerType() const;
    float getMaxTorque() const;
    float getAverageTorque() const;

    /** @brief Override comparison operator */
    friend bool operator==(const TorqueSensor& lhs, const TorqueSensor& rhs)
    {
        return lhs.getMotorControllerType() == rhs.getMotorControllerType() && lhs.max_torque_ == rhs.max_torque_
            && lhs.average_torque_ == rhs.average_torque_;
    }
    /** @brief Override stream operator for clean printing */
    friend std::ostream& operator<<(std::ostream& os, const TorqueSensor& torque_sensor)
    {
        return os << "max_torque: " << torque_sensor.max_torque_;
    }

private:
    // Type of motorcontroller that is uses the sensor.
    // (now we only have ODrives, but makes changing to different controllers easier in the future).
    const MotorControllerType motor_controller_type_;

    // Max torque that the may outputted at any time, if outputted torque is higher a warning/ error occurs
    const float max_torque_;
    const float average_torque_;
};
} // namespace march

#endif // BUILD_TORQUE_SENSOR_HPP
