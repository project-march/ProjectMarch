//
// Created by martijn on 8-2-19.
//

#ifndef PROJECT_PDB_H
#define PROJECT_PDB_H

#include "Slave.h"

class PDB : public Slave
{
public:
  PDB(std::string name, uint16 number);
  void publish();
};

#endif  // PROJECT_PDB_H
