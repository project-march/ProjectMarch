#pragma once
#include "march_hardware/communication/ethercat/sdo_interface.h"

#include <memory>

#include <gmock/gmock.h>

class MockSdoInterface : public march::SdoInterface
{
public:
  MockSdoInterface() = default;

protected:
  MOCK_METHOD5(write, int(uint16_t, uint16_t, uint8_t, std::size_t, void*));

  MOCK_CONST_METHOD5(read, int(uint16_t, uint16_t, uint8_t, int&, void*));
};

using MockSdoInterfacePtr = std::shared_ptr<MockSdoInterface>;
