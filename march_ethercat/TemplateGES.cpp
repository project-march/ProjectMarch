//
// Created by Martijn on 8-2-19.
//

#include "TemplateGES.h"

TemplateGES::TemplateGES(std::string name, uint16 number) : GES(name, number)
{
  type = "TEMPLATEGES";
  offset = 0;
}

void TemplateGES::setLedCommand(uint8 value)
{
  union bit8 unionbyte;
  unionbyte.ui = value;
  set_output_bit8(number, offset, unionbyte);
  //  printf("Set LED\n");
}

void TemplateGES::publish()
{
  int8 LedAck = get_input_bit8(number, offset).ui;
  printf("Returned value: %d\n", LedAck);
}