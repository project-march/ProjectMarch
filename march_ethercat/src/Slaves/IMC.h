//
// Class header for IMotionCubes
// Derived from Slave class
//

#ifndef PROJECT_IMC_H
#define PROJECT_IMC_H

#include "Slave.h"

class IMC : public Slave
{
public:
  // Constructor
  IMC(std::string name, uint16 number);
  // Process Data Object mapping
  bool PDOmapping();
  // Set configuration parameters for the IMC
  bool StartupSDO(int ecatCycleTime);
  // Publish received data
  void publish();
};

#endif  // PROJECT_IMC_H
