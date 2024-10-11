/*
Authors: Femke Buiks, MIX

This is the header file for the GaitPlanningAngles class. 

*/ 

#include "march_shared_msgs/msg/exo_mode.hpp"
#include "../../march_mode_machine/include/march_mode_machine/exo_mode.hpp"
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>  

const int RIGHT_STANCE_LEG = 2; 
const int LEFT_STANCE_LEG = 1; 
const int DOUBLE_STANCE_LEG = 3; 

class GaitPlanningAngles{
    public:
    explicit GaitPlanningAngles(); 
 
    void processCSVFile(const std::string &path, std::vector<std::vector<double>> &member_variable);

    // Setters
    void setGaitType(const ExoMode &new_gait_type); 
    void setPrevGaitType(const ExoMode &prev_gait_type); 
    void setCounter(const int &count); 
    void setPrevPoint(const std::vector<double> &point); 
    void setHomeStand(const std::vector<double> &stand); 
    void setStanceFoot(const uint8_t &foot); 
    void setHipTilt(const double &hip_tilt); 

    // Getters
    ExoMode getGaitType() const; 
    ExoMode getPrevGaitType() const; 
    std::vector<double> getPrevPoint() const; 
    int getCounter() const;
    std::vector<std::vector<double>> getFirstStepAngleTrajectory() const; 
    std::vector<std::vector<double>> getFullGaitAngleCSV() const; 
    std::vector<std::vector<double>> getStandToSitGait() const; 
    std::vector<std::vector<double>> getSidewaysRightGait() const; 
    std::vector<std::vector<double>> getSidewaysLeftGait() const;
    std::vector<std::vector<double>> getSitToStandGait() const; 
    std::vector<std::vector<double>> getStepCloseGait() const; 
    std::vector<std::vector<double>> getHingeGait() const;
    std::vector<std::vector<double>> getStandToTrainSitGait() const;
    std::vector<std::vector<double>> getTrainSitToStandGait() const;
    std::vector<double> getHomeStand() const; 
    uint8_t getStanceFoot() const; 
    double getHipTilt() const; 

    std::vector<std::vector<double>> swapLeftAndRightColumns(const std::vector<std::vector<double>>& matrix) const; 
    
    private: 
    ExoMode m_gait_type; 
    ExoMode m_prev_gait_type; 
    std::vector<std::vector<double>> m_first_step_angle_trajectory; 
    std::vector<std::vector<double>> m_complete_step_angle_trajectory; 
    std::vector<std::vector<double>> m_half_step_angle_trajectory; 
    std::vector<std::vector<double>> m_stand_to_sit_trajectory; 
    std::vector<std::vector<double>> m_sideways_right_trajectory; 
    std::vector<std::vector<double>> m_sideways_left_trajectory;
    std::vector<std::vector<double>> m_sit_to_stand_trajectory; 
    std::vector<std::vector<double>> m_step_close_trajectory; 
    std::vector<std::vector<double>> m_hinge_trajectory; 
    std::vector<std::vector<double>> m_stand_to_train_sit_trajectory;
    std::vector<std::vector<double>> m_train_sit_to_stand_trajectory;
    std::vector<double> m_home_stand; 
    std::vector<double> m_prev_point; 
    double m_hip_tilt; 
    int m_counter; 
    uint8_t m_stance_foot; 
};