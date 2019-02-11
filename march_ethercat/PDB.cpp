//
// Created by martijn on 8-2-19.
//

#include "PDB.h"

PDB::PDB(std::string name, uint16 number) : Slave(name, number)
{
  type = "PDB";
}

void PDB::publish()
{
}