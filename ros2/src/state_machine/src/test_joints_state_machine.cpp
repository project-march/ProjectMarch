/*Authors: Andrew Hutani, MIX

This class is a seperate StateMachine for the testing of single joints. It is a child class of the normal StateMachine class, where only the constructor (i.e. the initialization of the transitions) is different.

This class is only used in the test_joints launch file.

*/

#include "state_machine/test_joints_state_machine.hpp"
#include <set>
#include <map>

TestJointsStateMachine::TestJointsStateMachine()
{
        m_exo_transitions ={
                /*{CurrentState, PossibleStates}*/
                {{exoState::BootUp, {exoState::Stand}},
                {exoState::Stand, {exoState::Walk, exoState::BootUp}},
                {exoState::Walk, {exoState::Stand}}}};

        setCurrentState(exoState::BootUp);
        RCLCPP_WARN(rclcpp::get_logger("state_machine"), "Test joints State Machine created");
}

