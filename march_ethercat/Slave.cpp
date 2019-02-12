//
// Base class source for all slaves
//

#include "Slave.h"

// Constructor
Slave::Slave(std::string name, uint16 number)
{
  this->name = name;
  this->number = number;
  type = "SLAVE";
}

// Getters

std::string Slave::getName()
{
  return name;
}

int Slave::getNumber()
{
  return number;
}

std::string Slave::getType()
{
  return type;
}