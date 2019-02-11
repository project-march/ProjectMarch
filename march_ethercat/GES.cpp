//
// Created by Martijn on 5-2-19.
//

#include "GES.h"

GES::GES(std::string name, uint16 number) : Slave(name, number)
{
  type = "GES";
}
