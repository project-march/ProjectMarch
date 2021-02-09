#pragma once
#include "march_hardware/communication/ethercat/pdo_interface.h"
#include "march_hardware/communication/ethercat/pdo_types.h"

#include <memory>

#include <gmock/gmock.h>

class MockPdoInterface : public march::PdoInterface
{
public:
  MockPdoInterface() = default;

  MOCK_METHOD3(write8, void(uint16_t, uint8_t, march::bit8));
  MOCK_METHOD3(write16, void(uint16_t, uint8_t, march::bit16));
  MOCK_METHOD3(write32, void(uint16_t, uint8_t, march::bit32));

  MOCK_CONST_METHOD2(read8, march::bit8(uint16_t, uint8_t));
  MOCK_CONST_METHOD2(read16, march::bit16(uint16_t, uint8_t));
  MOCK_CONST_METHOD2(read32, march::bit32(uint16_t, uint8_t));
};

using MockPdoInterfacePtr = std::shared_ptr<MockPdoInterface>;
