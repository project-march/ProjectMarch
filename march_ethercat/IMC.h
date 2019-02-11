//
// Created by Martijn on 8-2-19.
//

#ifndef PROJECT_IMC_H
#define PROJECT_IMC_H

#include "Slave.h"

class IMC : public Slave
{
public:
  IMC(std::string name, uint16 number);
  bool PDOmapping();
  bool StartupSDO(int ecatCycleTime);
  void publish();
};

#endif  // PROJECT_IMC_H
