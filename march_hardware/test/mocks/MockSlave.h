#pragma once
#include "MockPdoInterface.h"
#include "MockSdoInterface.h"

#include "march_hardware/EtherCAT/slave.h"

class MockSlave : public march::Slave
{
public:
  MockSlave() : Slave(1, mock_pdo, mock_sdo){};

  MockPdoInterface mock_pdo;
  MockSdoInterface mock_sdo;
};
