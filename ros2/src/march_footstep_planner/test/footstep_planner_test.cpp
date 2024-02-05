#include <gtest/gtest.h>
#include "march_footstep_planner/footstep_planner.hpp"
#include "march_footstep_planner/footstep_planner_node.hpp"
#include "march_shared_msgs/msg/plane.hpp"

class FootstepPlannerTest : public ::testing::Test {
    public: 
    FootstepPlannerTest() = default; 
    ~FootstepPlannerTest() override = default; 
    protected: 
    void SetUp(){
        m_footstep_planner = std::make_unique<FootstepPlanner>(); 
    }
    std::unique_ptr<FootstepPlanner> m_footstep_planner; 
};

TEST_F(FootstepPlannerTest, test_compare_distance){
    march_shared_msgs::msg::Plane plane1; 
    march_shared_msgs::msg::Plane plane2; 
    m_footstep_planner.setFootPositions({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}); 
    plane1.centroid.x = 0.1; 
    plane2.centroid.x = 0.2; 
    ASSERT_TRUE(compareDistance(plane1, plane2)); 
}

// TEST_F(FootstepPlannerTest, test_check_centroid_function)

// TEST_F(FootstepPlannerTest, test_find_safe_plane_function)