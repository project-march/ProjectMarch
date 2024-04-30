/*
Author: Femke Buiks, MIX
*/

/*
From paper with slight adaptations: 
- determine center point and orientation of all planes [NODE]
- rank planes by distance w/ smallest distance to baseframe first [NODE]
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

#pragma once

#include <array>
#include <vector>
#include <iostream>
#include "geometry_msgs/msg/point.hpp"
#include "march_shared_msgs/msg/plane.hpp"


class FootstepPlanner {
    public: 
    explicit FootstepPlanner(); 

    //Setters
    // void setFootPositions(const double &l_x, const double &l_y, const double &l_z, const double &r_x, const double &r_y, const double &r_z); 
    void setFootSize(const double &width, const double &length, const double &height); 
    void setDistanceThreshold(const double &distance); 
    void setFootPositions(const std::array<double, 3> &new_left_foot_position, const std::array<double, 3> &new_right_foot_position);
    void setPlanesList(const std::vector<march_shared_msgs::msg::Plane> &planes);  
    //Getters
    std::array<double, 3> getFootSize() const;
    double getDistanceThreshold() const; 
    std::array<double, 3> getLeftFootPosition() const; 
    std::array<double, 3> getRightFootPosition() const; 
    std::vector<march_shared_msgs::msg::Plane> getPlanesList() const; 

    bool compareDistance(const geometry_msgs::msg::Point& plane1_centroid, const geometry_msgs::msg::Point& plane2_centroid) const; 
    bool checkCentroidPlaneSafeDistance(const march_shared_msgs::msg::Plane& plane) const; 
    void rankPlanesByDistance(); 
    march_shared_msgs::msg::Plane& findSafePlane(size_t index = 0); 
    bool checkOverlapPlaneFootbox(const march_shared_msgs::msg::Plane& plane) const;
    bool checkIfCircle(const march_shared_msgs::msg::Plane& plane) const; 

    private:

    double m_step_distance_threshold; 
    std::array<double, 3> m_foot_size; 
    std::array<double, 3> m_left_foot_position; 
    std::array<double, 3> m_right_foot_position; 
    std::vector<march_shared_msgs::msg::Plane> m_planes_list; 

}; 