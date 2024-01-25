/*Authors: Femke Buiks and Andrew Hutani, MIX*/

#pragma once

#include <string>
#include <vector>
#include <array>  
#include "../../march_mode_machine/include/march_mode_machine/exo_mode.hpp"

class GaitPlanning {
    
public:
    explicit GaitPlanning();

    // Setters
    void setStanceFoot(const uint8_t &new_stance_foot);
    void setFootPositions(const std::array<double, 3> &new_left_foot_position, const std::array<double, 3> &new_right_foot_position);
    void setGaitType(const exoMode &new_gait_type);  
    void setBezierGait();

    // Getters
    std::vector<std::array<double, 4>> getTrajectory() const;
    int getCurrentStanceFoot() const; 
    std::array<double, 3> getCurrentLeftFootPos() const; 
    std::array<double, 3> getCurrentRightFootPos() const; 
    exoMode getGaitType() const; 

    //Getter for variable step size 
    std::vector<std::array<double, 4>> getVariableTrajectory() const; 

    std::vector<double> linspace(const double &min, const double &max, const int &size); 

    //Interpolate between small and large bezier gait 
    std::vector<std::array<double, 4>> interpolateVariableTrajectory(const float &step_distance); 

private: 
    exoMode m_gait_type; 
    int m_current_stance_foot; 
    double m_step_size; 
    std::array<double, 3> m_current_left_foot_position; 
    std::array<double, 3> m_current_right_foot_position; 
    std::vector<std::array<double, 4>> m_large_bezier_trajectory; 
    std::vector<std::array<double, 4>> m_large_first_step_trajectory; 
    std::vector<std::array<double, 4>> m_small_bezier_trajectory; 
    std::vector<std::array<double, 4>> m_small_first_step_trajectory; 
    
    //Create trajectory for variable step size. This should already include a stepclose. 
    std::vector<std::array<double, 4>> m_variable_step_trajectory; 

    std::vector<std::array<double, 4>> processCSV(const std::string& filename);
    
}; 