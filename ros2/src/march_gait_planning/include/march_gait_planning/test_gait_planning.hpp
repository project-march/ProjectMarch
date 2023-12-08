#ifndef TEST_GAIT_PLANNING_HPP
#define TEST_GAIT_PLANNING_HPP

#include "march_shared_msgs/msg/exo_state.hpp"
#include <vector>
#include <array>
#include "../../state_machine/include/state_machine/exo_state.hpp"

class TestGaitPlanning
{
public:
    TestGaitPlanning();
    void setTrajectory();
    std::vector<double> getTrajectory() const;
    void setGaitType(const exoState &new_gait_type);
    exoState getGaitType() const;

private:
    exoState m_gait_type;
    std::vector<double> m_trajectory;
};

#endif // TEST_GAIT_PLANNING_HPP