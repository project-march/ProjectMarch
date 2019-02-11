//
// Created by Martijn on 8-2-19.
//

#ifndef PROJECT_TEMPLATEGES_H
#define PROJECT_TEMPLATEGES_H

#include "GES.h"

class TemplateGES : public GES
{
private:
  uint8 offset;

public:
  TemplateGES(std::string name, uint16 number);
  void setLedCommand(uint8 value);
  void publish();
};

#endif  // PROJECT_TEMPLATEGES_H
