#include <gtest/gtest.h>
#include "march_footstep_planner/footstep_planner.hpp"

class FootstepPlannerTest : public ::testing::Test {
    public: 
    FootstepPlannerTest() = default; 
    ~FootstepPlannerTest() override = default; 
    protected: 
    void SetUp(){
        m_footstep_planner = std::make_unique<FootstepPlanner>(); 
        m_footstep_planner->setFootPositions({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}); 
        m_footstep_planner->setDistanceThreshold(0.6); 
        m_footstep_planner->setFootSize(30, 25, 0); 
        plane1.centroid = geometry_msgs::build<geometry_msgs::msg::Point>().x(0.5).y(1.0).z(0.0); 
        plane2.centroid = geometry_msgs::build<geometry_msgs::msg::Point>().x(1.0).y(5.0).z(0.0); 
        plane3.centroid = geometry_msgs::build<geometry_msgs::msg::Point>().x(1.5).y(0.0).z(0.0);  
        plane1.upper_boundary_point.x = 30.0; 
        plane1.lower_boundary_point.x = -30.0; 
        plane1.left_boundary_point.y = 30.0;
        plane1.right_boundary_point.y = -30.0;

        plane2.upper_boundary_point.x = 10.0; 
        plane2.lower_boundary_point.x = -10.0; 
        plane2.left_boundary_point.y = 8.0;
        plane2.right_boundary_point.y = -8.0; 
    }
    std::unique_ptr<FootstepPlanner> m_footstep_planner; 
    march_shared_msgs::msg::Plane plane1; 
    march_shared_msgs::msg::Plane plane2;
    march_shared_msgs::msg::Plane plane3; 
};

TEST_F(FootstepPlannerTest, test_compare_distance_good){
    ASSERT_TRUE(m_footstep_planner->compareDistance(plane1.centroid, plane2.centroid)); 
}

TEST_F(FootstepPlannerTest, test_compare_distance_bad) {
    ASSERT_FALSE(m_footstep_planner->compareDistance(plane2.centroid, plane1.centroid));
}

TEST_F(FootstepPlannerTest, test_check_centroid_plane_good) {
    ASSERT_TRUE(m_footstep_planner->checkCentroidPlaneSafeDistance(plane1)); 
}

TEST_F(FootstepPlannerTest, test_check_centroid_plane_bad) {
    ASSERT_FALSE(m_footstep_planner->checkCentroidPlaneSafeDistance(plane2)); 
}

TEST_F(FootstepPlannerTest, test_rank_planes_many) {
    std::vector<march_shared_msgs::msg::Plane> wrong_order = {plane2, plane1, plane3}; 
    std::vector<march_shared_msgs::msg::Plane> right_order = {plane1, plane2, plane3}; 
    m_footstep_planner->setPlanesList(wrong_order); 
    m_footstep_planner->rankPlanesByDistance(); 
    ASSERT_EQ(m_footstep_planner->getPlanesList(), right_order); 
}

TEST_F(FootstepPlannerTest, test_find_safe_plane){
    m_footstep_planner->setPlanesList({plane2, plane1, plane3}); 
    march_shared_msgs::msg::Plane& res = m_footstep_planner->findSafePlane(); 
    ASSERT_EQ(res, plane1); 
}

TEST_F(FootstepPlannerTest, test_overlap_footbox_good) {
    ASSERT_TRUE(m_footstep_planner->checkOverlapPlaneFootbox(plane1)); 
}

TEST_F(FootstepPlannerTest, test_overlap_footbox_bad) {
    ASSERT_FALSE(m_footstep_planner->checkOverlapPlaneFootbox(plane2)); 
}

TEST_F(FootstepPlannerTest, test_if_circle_good){
    ASSERT_TRUE(m_footstep_planner->checkIfCircle(plane1)); 
}

TEST_F(FootstepPlannerTest, test_if_circle_bad){
    ASSERT_FALSE(m_footstep_planner->checkIfCircle(plane2)); 
}