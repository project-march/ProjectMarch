// Copyright 2019 Project March.
#ifndef PROJECT_TEMPERATUREINTERFACE_H
#define PROJECT_TEMPERATUREINTERFACE_H

namespace march4cpp
{
class TemperatureSensor
{
 public:
  virtual float getTemperature() = 0;
};
}  // namespace march4cpp

#endif  // PROJECT_TEMPERATUREINTERFACE_H
