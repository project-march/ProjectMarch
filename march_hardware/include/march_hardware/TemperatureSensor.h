// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_TEMPERATURESENSOR_H
#define MARCH_HARDWARE_TEMPERATURESENSOR_H

namespace march
{
class TemperatureSensor
{
public:
  virtual float getTemperature() = 0;
};
}  // namespace march

#endif  // MARCH_HARDWARE_TEMPERATURESENSOR_H
