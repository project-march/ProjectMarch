#ifndef TEST_GAIT_PLANNING_HPP
#define TEST_GAIT_PLANNING_HPP

#include <vector>
#include <array>
#include <string>
#include "../../march_mode_machine/include/march_mode_machine/exo_mode.hpp"

class TestJointsGaitPlanning
{
public:
    TestJointsGaitPlanning();
    void loadTrajectoryFromCSV(const std::string &path, std::vector<double> &member_variable);
    std::vector<double> getTrajectory(int& actuated_joint) const;
    void setGaitType(const exoMode &new_gait_type);
    exoMode getGaitType() const;

    std::vector<double> getPrevPoint() const; 
    void setPrevPoint(const std::vector<double> &point);

private:
    exoMode m_gait_type;
    std::vector<double> m_trajectory_adpf;
    std::vector<double> m_trajectory_haa;
    std::vector<double> m_trajectory_hfe;
    std::vector<double> m_trajectory_kfe;

    std::vector<double> m_prev_point; 
};

#endif // TEST_GAIT_PLANNING_HPP