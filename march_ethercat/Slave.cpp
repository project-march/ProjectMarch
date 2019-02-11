//
// Created by Martijn on 5-2-19.
//

#include "Slave.h"

Slave::Slave(std::string name, uint16 number)
{
  this->name = name;
  this->number = number;
  type = "SLAVE";
}

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