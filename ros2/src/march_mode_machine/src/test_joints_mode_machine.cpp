/*Authors: Andrew Hutani, MIX

This class is a seperate ModeMachine for the testing of single joints. It is a child class of the normal ModeMachine class, where only the constructor (i.e. the initialization of the transitions) is different.

This class is only used in the test_joints launch file.

*/

#include "march_mode_machine/test_joints_mode_machine.hpp"
#include <set>
#include <map>

TestJointsModeMachine::TestJointsModeMachine()
{
        m_exo_transitions = ExoModeTransitions("Test Joints");

        setCurrentMode(ExoMode::BootUp);
        RCLCPP_WARN(rclcpp::get_logger("mode_machine"), "Test joints Mode Machine created");
}

