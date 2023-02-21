#ifndef __clang_analyzer__
// NOLINTBEGIN

//#include "state_machine.cpp"
#include "state_machine.hpp"
#include <gtest/gtest.h>
#include <memory>

class StateMachineTest : public testing::Test {
protected:
    std::unique_ptr<StateMachine> state_machine;

private:
    void SetUp() override
    {
        state_machine = std::make_unique<StateMachine>();
    }
};

TEST_F(StateMachineTest, getCurrentStateTest)
{
    ASSERT_EQ(this->state_machine->get_current_state(), exoState::ForceUnknown);
}

TEST_F(StateMachineTest, performTransitionSucces)
{
    this->state_machine->performTransition(exoState::ForceUnknownStand);
    exoState expectedState = exoState::ForceUnknownStand;
    ASSERT_EQ(this->state_machine->get_current_state(), expectedState);
}

TEST_F(StateMachineTest, performTransitionToError)
{
    this->state_machine->performTransition(exoState::Walk);
    exoState expectedState = exoState::Error;
    ASSERT_EQ(this->state_machine->get_current_state(), expectedState);
}

// NOLINTEND
#endif
