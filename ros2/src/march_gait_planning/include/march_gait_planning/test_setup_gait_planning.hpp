#ifndef TEST_GAIT_PLANNING_HPP
#define TEST_GAIT_PLANNING_HPP

#include <vector>
#include <array>
#include <string>
#include "../../march_mode_machine/include/march_mode_machine/exo_mode.hpp"
#include "march_gait_planning/gait_planning_base_class.hpp"

class TestSetupGaitPlanning : public GaitPlanning
{
public:
    TestSetupGaitPlanning();
    void loadTrajectoryFromCSV();
    std::vector<double> getTrajectory() const;


private:
    std::vector<double> m_trajectory;
};

#endif // TEST_GAIT_PLANNING_HPP