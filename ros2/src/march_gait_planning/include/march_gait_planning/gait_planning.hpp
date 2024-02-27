/*Authors: Femke Buiks and Andrew Hutani, MIX*/

#pragma once

#include <string>
#include <vector>
#include <array>  
#include "../../march_mode_machine/include/march_mode_machine/exo_mode.hpp"

class GaitPlanning {
    
public:
    explicit GaitPlanning();
    typedef std::array<double, 4> XZFeetPositionsArray;
    typedef std::array<double, 3> XYZFootPositionArray; 

    // Setters
    void setStanceFoot(const uint8_t &new_stance_foot);
    void setFootPositions(const XYZFootPositionArray &new_left_foot_position, const XYZFootPositionArray &new_right_foot_position);
    void setGaitType(const exoMode &new_gait_type);
    void setPreviousGaitType(const exoMode &previous_gait_type);  
    void setBezierGait();

    // Getters
    std::vector<XZFeetPositionsArray> getTrajectory() const;
    int getCurrentStanceFoot() const; 
    XYZFootPositionArray getCurrentLeftFootPos() const; 
    XYZFootPositionArray getCurrentRightFootPos() const; 
    exoMode getGaitType() const; 
    exoMode getPreviousGaitType() const;

    //Getter for variable step size 
    std::vector<XZFeetPositionsArray> getVariableTrajectory() const; 

    std::vector<double> linspace(const double &min, const double &max, const int &size); 

    //Interpolate between small and large bezier gait 
    std::vector<XZFeetPositionsArray> interpolateVariableTrajectory(const float &step_distance); 

private: 
    exoMode m_gait_type; 
    exoMode m_previous_gait_type;
    int m_current_stance_foot; 
    double m_step_size; 
    
    XYZFootPositionArray m_current_left_foot_position; 
    XYZFootPositionArray m_current_right_foot_position; 

    // Order: x_swing, z_swing, x_stance, z_stance
    std::vector<XZFeetPositionsArray> m_large_bezier_trajectory; 
    std::vector<XZFeetPositionsArray> m_large_first_step_trajectory; 
    std::vector<XZFeetPositionsArray> m_small_bezier_trajectory; 
    std::vector<XZFeetPositionsArray> m_small_first_step_trajectory; 
    std::vector<XZFeetPositionsArray> m_large_step_close_trajectory;
    std::vector<XZFeetPositionsArray> m_small_step_close_trajectory;
    std::vector<XZFeetPositionsArray> m_high_step_26cm_trajectory;
    std::vector<XZFeetPositionsArray> m_high_step_26cm_close_trajectory;
    
    //Create trajectory for variable step size. This should already include a stepclose. 
    std::vector<XZFeetPositionsArray> m_variable_step_trajectory; 

    std::vector<XZFeetPositionsArray> processCSV(const std::string& filename);
    
}; 