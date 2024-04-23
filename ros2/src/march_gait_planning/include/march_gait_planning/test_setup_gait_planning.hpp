#ifndef TEST_GAIT_PLANNING_HPP
#define TEST_GAIT_PLANNING_HPP

#include <vector>
#include <array>
#include <string>
#include "../../march_mode_machine/include/march_mode_machine/exo_mode.hpp"

class TestSetupGaitPlanning
{
public:
    TestSetupGaitPlanning();
    void loadTrajectoryFromCSV();
    std::vector<double> getTrajectory() const;
    void setGaitType(const exoMode &new_gait_type);
    exoMode getGaitType() const;

private:
    exoMode m_gait_type;
    std::vector<double> m_trajectory;
};

#endif // TEST_GAIT_PLANNING_HPP