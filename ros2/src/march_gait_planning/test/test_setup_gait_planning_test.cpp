#include "gtest/gtest.h"
#include "march_gait_planning/test_setup_gait_planning.hpp"

class TestSetupGaitPlanningTest : public testing::Test {
    protected: 
    TestSetupGaitPlanning test_setup_gait_planning = TestSetupGaitPlanning();
};

TEST_F(TestSetupGaitPlanningTest, AssertObjectCreated){
    // assert creation of a gait planning object contains all necessary member variables
}

TEST_F(TestSetupGaitPlanningTest, LoadTrajectoryCorrectly) {
    // Act
    test_setup_gait_planning.loadTrajectoryFromCSV();
    std::vector<double> trajectory = test_setup_gait_planning.getTrajectory();

    // Assert
    EXPECT_NEAR(trajectory[50], 0, 1e-6);  // Check that the middle value is close to 0 (since the trajectory is a sine wave)
}

TEST_F(TestSetupGaitPlanningTest, setGaitTypeCorrectly){
    // Act
    ExoMode state = ExoMode::Walk;
    test_setup_gait_planning.setGaitType(state);

    // Assert
    ASSERT_EQ(state, test_setup_gait_planning.getGaitType());
}
