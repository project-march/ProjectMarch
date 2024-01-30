/*
From paper with slight adaptations: 
- determine center point and orientation of all planes [NODE]
- rank planes by distance w/ smallest distance to baseframe first, 
and filter for larger possible stepping box [NODE]
- check y of centroid of first plane in list ( in paper to determine swing foot, 
but in our case we always step with right first so check if y is close enough for step)
- (check yaw of plane? our planes will most likely be large enough or 
always be located directly in front so I don't think this is necessary)
- draw rectangle size of foot (or 2 feet, dependent on how we want to step 
on stairs/stepping stones) around centroid, check how many points of the 
rectangle overlap with points of plane. If above threshold, this plane is 
safe to step in. 
    - the above can be done for multiple points, ultimately the rectangle 
    with largest number of overlapping points should be stepped towards. 
- send coordinates of desired centroid stepping point to gait planning 
(check if we use ankle or toe as x in IKS, that might make a difference)
*/

#include <vector>
#include "rclcpp/rclcpp.hpp" 
#include "march_footstep_planner/include/footstep_planner.hpp"
#include "march_shared_msgs/msg/foot_step_output.hpp"
#include "../../march_mode_machine/include/march_mode_machine/exo_mode.hpp"


class FootstepPlannerNode:public rclcpp::Node {
    public: 
    explicit FootstepPlannerNode(); 

    private: 

    rclcpp::Publisher<march_shared_msgs::msg::FootStepOutput>::SharedPtr m_variable_footstep_publisher; 
    
    //Subscribe to output from cams, used Planes as dummy message for now.  
    rclcpp::Subscription<march_shared_msgs::msg::Planes>::SharedPtr m_planes_subscriber; 
    rclcpp::Subscription<march_shared_msgs::msg::ExoMode>::SharedPtr m_exo_mode_subscriber;
    // subscribe to current exo state as we need the offset of the feet to generate coordinates w/ respect to baseframe
    rclcpp::subscription<march_shared_msgs::msg::StateEstimation>::SharedPtr m_exo_joint_state_subscriber;  

    void planesCallback(const march_shared_msgs::msg::Planes::SharedPtr msg);
    void currentModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg);
    void currentExoStateCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg); 

    void footstepOutputPublish(); 

    // Used planes as dummy object data type for now
    bool compareDistance(const plane plane1, const plane plane2) const; 
    void rankPlanesByDistance(); 
    bool checkCentroidPlaneSafe(const plane plane) const; 
    plane findSafePlane(size_t index = 0); 
    bool checkOverlapPlaneFootbox();
    void selectDesiredPoint(); 
    

    std::vector<plane> m_planes_list; 
    std::array<double, 3> m_desired_point; 
    march_shared_msgs::msg::FootStepOutput::SharedPtr m_desired_footstep_msg; 
    exoMode m_gait_type; 

    FootstepPlanner m_footstep_planner; 



}