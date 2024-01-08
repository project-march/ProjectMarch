/*
Authors: Femke Buiks, MIX

This is the header file for the GaitPlanningAngles class. 

*/ 

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
 
    void processCSVFile(const std::string &path, std::vector<std::vector<double>> &member_variable);

    // Setters
    void setGaitType(const exoState &new_gait_type); 
    void setPrevGaitType(const exoState &prev_gait_type); 
    void setCounter(const int &count); 
    void setPrevPoint(const std::vector<double> &point); 
    void setHomeStand(const std::vector<double> &stand); 

    // Getters
    exoState getGaitType() const; 
    exoState getPrevGaitType() const; 
    std::vector<double> getPrevPoint() const; 
    int getCounter() const;
    std::vector<std::vector<double>> getFirstStepAngleTrajectory() const; 
    std::vector<std::vector<double>> getFullGaitAngleCSV() const; 
    std::vector<std::vector<double>> getStandToSitGait() const; 
    std::vector<std::vector<double>> getSidewaysGait() const; 
    std::vector<std::vector<double>> getSitToStandGait() const; 
    std::vector<double> getHomeStand() const; 
    
    private: 
    exoState m_gait_type; 
    exoState m_prev_gait_type; 
    std::vector<std::vector<double>> m_first_step_angle_trajectory; 
    std::vector<std::vector<double>> m_complete_step_angle_trajectory; 
    std::vector<std::vector<double>> m_stand_to_sit_trajectory; 
    std::vector<std::vector<double>> m_sideways_trajectory; 
    std::vector<std::vector<double>> m_sit_to_stand_trajectory; 
    std::vector<double> m_home_stand; 
    std::vector<double> m_prev_point; 
    int m_counter; 
};