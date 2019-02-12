//
// Class header for general-purpose GES
// Derived from Slave class
//

#ifndef PROJECT_GES_H
#define PROJECT_GES_H

#include "Slave.h"

class GES : public Slave
{
public:
  // Constructor
  GES(std::string name, uint16 number);
  // Publish received data
  virtual void publish() = 0;
};

#endif  // PROJECT_GES_H
