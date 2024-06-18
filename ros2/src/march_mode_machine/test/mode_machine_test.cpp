#ifndef __clang_analyzer__
// NOLINTBEGIN

//#include "mode_machine.cpp"
#include "mode_machine.hpp"
#include <gtest/gtest.h>
#include <memory>

class ModeMachineTest : public testing::Test {
protected:
    std::unique_ptr<ModeMachine> mode_machine;

private:
    void SetUp() override
    {
        mode_machine = std::make_unique<ModeMachine>();
    }
};

TEST_F(ModeMachineTest, getCurrentModeTest)
{
    ASSERT_EQ(this->mode_machine->getCurrentMode(), (int)ExoMode::BootUp);
}

TEST_F(ModeMachineTest, performTransitionSucces)
{
    this->mode_machine->performTransition(ExoMode::Stand);
    ExoMode expectedMode = ExoMode::Stand;
    ASSERT_EQ(this->mode_machine->getCurrentMode(), (int)expectedMode);
}

TEST_F(ModeMachineTest, performInvalidTansitionModeDontChange)
{
    this->mode_machine->performTransition(ExoMode::Walk);
    ExoMode expectedMode = ExoMode::BootUp;
    ASSERT_EQ(this->mode_machine->getCurrentMode(), (int)expectedMode);
}

// NOLINTEND
#endif
