#include "march_input_device/input_device.hpp"
#include <gtest/gtest.h>
#include <memory>

using testing::Eq;

class InputDeviceTest : public testing::Test {
protected:
std::unique_ptr<IPD> input_device_test;
private:
void SetUp() override
{
input_device_test = std::make_unique<IPD>();
}
};
// class InputDeviceTest : public testing::Test {
//     protected: 
//     IPD input_device_test = IPD();
// };


TEST_F(InputDeviceTest, setStateTest)
{ 
ASSERT_EQ(input_device_test->getCurrentState(), exoState::BootUp); 
}

TEST_F(InputDeviceTest, getAvailableTest)
{
input_device_test->setCurrentState(exoState::BootUp);
std::set<exoState> expected_states = {exoState::Stand, exoState::Sit, exoState::Error, exoState::BootUp};
ASSERT_EQ(input_device_test->getAvailableStates(), expected_states); 
}
