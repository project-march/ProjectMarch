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
    void setGaitType(const ExoMode &new_gait_type);
    ExoMode getGaitType() const;

    std::vector<double> getPrevPoint() const; 
    void setPrevPoint(const std::vector<double> &point);

private:
    ExoMode m_gait_type;
    std::vector<double> m_trajectory;
    std::vector<double> m_prev_point; 
};

#endif // TEST_GAIT_PLANNING_HPP