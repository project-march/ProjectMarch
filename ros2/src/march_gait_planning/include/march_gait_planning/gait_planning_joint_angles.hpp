#include "march_shared_msgs/msg/exo_state.hpp"
#include "../../state_machine/include/state_machine/exo_state.hpp"
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <sstream> 

class GaitPlanningAngles{
    public:
    explicit GaitPlanningAngles(); 

    // Setters 
    void setFirstStepAngleCSV(); 
    void setFullGaitAngleCSV(); 
    void setGaitType(const exoState &new_gait_type); 
    void setPrevGaitType(const exoState &prev_gait_type); 
    void setCounter(const int &count); 
    void setPrevPoint(const std::vector<double> &point); 

    // Getters
    exoState getGaitType() const; 
    exoState getPrevGaitType() const; 
    std::vector<double> getPrevPoint() const; 
    int getCounter() const;
    std::vector<std::vector<double>> getFirstStepAngleTrajectory() const; 
    std::vector<std::vector<double>> getFullGaitAngleCSV() const; 

    private: 
    exoState m_gait_type; 
    exoState m_prev_gait_type; 
    std::vector<std::vector<double>> m_first_step_angle_trajectory; 
    std::vector<std::vector<double>> m_complete_step_angle_trajectory; 
    std::vector<double> m_prev_point; 
    int m_counter; 
};