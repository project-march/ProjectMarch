// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_TEMPERATURE_SENSOR_H
#define MARCH_HARDWARE_TEMPERATURE_SENSOR_H

namespace march {
class TemperatureSensor {
public:
    virtual ~TemperatureSensor() noexcept = default;

    virtual float getTemperature() const = 0;
};
} // namespace march

#endif // MARCH_HARDWARE_TEMPERATURE_SENSOR_H
