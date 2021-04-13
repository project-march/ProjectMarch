// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_TEMPERATURE_GES_H
#define MARCH_HARDWARE_TEMPERATURE_GES_H
#include "march_hardware/ethercat/slave.h"
#include "temperature_sensor.h"

#include <cstddef>

namespace march {
class TemperatureGES : public Slave, TemperatureSensor {
public:
    TemperatureGES(const Slave& slave, uint8_t byte_offset);

    ~TemperatureGES() noexcept override = default;

    float getTemperature() const override;

    /** @brief Override comparison operator */
    friend bool operator==(const TemperatureGES& lhs, const TemperatureGES& rhs)
    {
        return lhs.getSlaveIndex() == rhs.getSlaveIndex()
            && lhs.temperature_byte_offset_ == rhs.temperature_byte_offset_;
    }
    /** @brief Override stream operator for clean printing */
    friend ::std::ostream& operator<<(
        std::ostream& os, const TemperatureGES& temperatureGes)
    {
        return os << "slaveIndex: " << temperatureGes.getSlaveIndex() << ", "
                  << "temperatureByteOffset: "
                  << temperatureGes.temperature_byte_offset_;
    }

private:
    const uint8_t temperature_byte_offset_;
};
} // namespace march
#endif // MARCH_HARDWARE_TEMPERATURE_GES_H
