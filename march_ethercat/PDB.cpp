//
// Class source for the PDB
// Derived from Slave class
//

#include "PDB.h"

// Constructor
PDB::PDB(std::string name, uint16 number) : Slave(name, number)
{
  type = "PDB";
}

// Publish received data
void PDB::publish()
{
  // Empty for now
}