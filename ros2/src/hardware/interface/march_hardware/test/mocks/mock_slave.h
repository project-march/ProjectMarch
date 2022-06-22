#pragma once
#include "mock_pdo_interface.h"
#include "mock_sdo_interface.h"

#include "march_hardware/ethercat/slave.h"

#include <memory>

class MockSlave : public march::Slave {
public:
    MockSlave()
        : Slave(/*slave_index=*/1, std::make_shared<MockPdoInterface>(),
            std::make_shared<MockSdoInterface>()) {};
    MockSlave(const MockPdoInterfacePtr& mock_pdo,
        const MockSdoInterfacePtr& mock_sdo)
        : Slave(/*slave_index=*/1, mock_pdo, mock_sdo) {};
};
