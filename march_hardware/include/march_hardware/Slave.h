#ifndef MARCH4CPP__SLAVE_H
#define MARCH4CPP__SLAVE_H

namespace march4cpp
{
class Slave
{
protected:
  int slaveIndex;

public:
  explicit Slave(int slaveIndex)
  {
    this->slaveIndex = slaveIndex;
  };

  Slave()
  {
    slaveIndex = -1;
  };

  ~Slave() = default;

  virtual void initialize() = 0;

  int getSlaveIndex()
  {
    return this->slaveIndex;
  }
};
}

#endif