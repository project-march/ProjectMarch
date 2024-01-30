#include <gtest/gtest.h>
#include "march_footstep_planner/include/footstep_planner.hpp"

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

// TEST_F(FootstepPlannerTest, test_compare_function)

// TEST_F(FootstepPlannerTest, test_check_centroid_function)

// TEST_F(FootstepPlannerTest, test_find_safe_plane_function)