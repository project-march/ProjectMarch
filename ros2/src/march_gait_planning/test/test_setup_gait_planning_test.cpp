#include "gtest/gtest.h"
#include "march_gait_planning/test_setup_gait_planning.hpp"

class TestSetupGaitPlanningTest : public testing::Test {
    protected: 
    TestSetupGaitPlanning test_gait_planning = TestSetupGaitPlanning();
};

TEST_F(TestSetupGaitPlanningTest, AssertObjectCreated){
    // assert creation of a gait planning object contains all necessary member variables
}
