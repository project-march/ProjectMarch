//
// Class source for GES used for testing purposes
// Derived from GES class
//

#include "TemplateGES.h"

// Constructor
TemplateGES::TemplateGES(std::string name, uint16 number) : GES(name, number)
{
  type = "TEMPLATEGES";
  LedCommandOffset = 0;
  LedAckOffset = 0;
}

// Set the LED command byte for the slave to the input value
void TemplateGES::setLedCommand(uint8 value)
{
  union bit8 unionbyte;
  unionbyte.ui = value;
  set_output_bit8(number, LedCommandOffset, unionbyte);
  //  printf("Set LED\n");
}

// Publish the received LED ack byte
void TemplateGES::publish()
{
  int8 LedAck = get_input_bit8(number, LedAckOffset).ui;
  // For now, publish by printing in terminal
  printf("Returned value: %d\n", LedAck);
}