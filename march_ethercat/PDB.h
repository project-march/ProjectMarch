//
// Class header for the PDB
// Derived from Slave class
//

#ifndef PROJECT_PDB_H
#define PROJECT_PDB_H

#include "Slave.h"

class PDB : public Slave
{
public:
  // Constructor
  PDB(std::string name, uint16 number);
  // Publish received data
  void publish();
};

#endif  // PROJECT_PDB_H
