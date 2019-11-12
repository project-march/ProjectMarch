// Copyright 2019 Project March

#ifndef HARDWARE_INTERFACE_MARCH_TEMPERATURE_SENSOR_INTERFACE_H
#define HARDWARE_INTERFACE_MARCH_TEMPERATURE_SENSOR_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>

namespace march_hardware_interface
{
class MarchTemperatureSensorHandle
{
public:
  MarchTemperatureSensorHandle(const std::string& name,    ///< The name of joint
                               const double* temperature,  ///< A pointer of the temperature in degrees celsius.
                               const double* variance      ///< A pointer to the storage of the temperature covariance.
                               )
    : name_(name), temperature_(temperature), variance(variance)
  {
  }

  std::string getName() const
  {
    return name_;
  }

  const double* getTemperature() const
  {
    return temperature_;
  }

  const double* getVariance() const
  {
    return variance;
  }

private:
  std::string name_;

  const double* temperature_;
  const double* variance;
};

class MarchTemperatureSensorInterface : public hardware_interface::HardwareResourceManager<MarchTemperatureSensorHandle>
{
};
}  // namespace march_hardware_interface

#endif  // HARDWARE_INTERFACE_MARCH_TEMPERATURE_SENSOR_INTERFACE_H