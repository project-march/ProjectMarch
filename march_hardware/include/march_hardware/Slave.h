#ifndef MARCH4CPP__SLAVE_H
#define MARCH4CPP__SLAVE_H

#include <stdexcept>
#include <ros/ros.h>

namespace march4cpp
{
class Slave
{
protected:
  int slaveIndex;

public:
  explicit Slave(int slaveIndex)
  {
    if (slaveIndex < 1)
    {
      ROS_FATAL("Slave configuration error: slaveindex can not be smaller than 1.");
      throw ::std::invalid_argument("Slave configuration error: slaveindex can not be smaller than 1.");
    }

    this->slaveIndex = slaveIndex;
  };

  Slave()
  {
    slaveIndex = -1;
  };

  ~Slave() = default;

  virtual void initialize() = 0;

  virtual int getSlaveIndex()
  {
    return this->slaveIndex;
  }
};
}

#endif