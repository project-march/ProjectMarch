//
// Class header for GES used for testing purposes
// Derived from GES class
//

#ifndef PROJECT_TEMPLATEGES_H
#define PROJECT_TEMPLATEGES_H

#include "GES.h"

class TemplateGES : public GES
{
private:
  uint8 LedCommandOffset;   // Bytes offset for the LED command
  uint8 LedAckOffset;       // Bytes offset for the LED ack
public:
  TemplateGES(std::string name, uint16 number);
  void setLedCommand(uint8 value);
  void publish();
};

#endif  // PROJECT_TEMPLATEGES_H
