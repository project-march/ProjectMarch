#include "march_gait_planning/gait_planning.hpp"
#include "gtest/gtest.h"

class GaitPlanningTest : public testing::Test {
    protected: 
    GaitPlanning test_gait_planning = GaitPlanning();
};

TEST_F(GaitPlanningTest, AssertObjectCreated){
    // assert creation of a gait planning object contains all necessary member variables
}

TEST_F(GaitPlanningTest, SetDoubleStanceCorrect){
    test_gait_planning.setStanceFoot(0);
    ASSERT_NE((int)test_gait_planning.getCurrentStanceFoot(), 1) << "Current stance foot is wrong"; 
    ASSERT_NE((int)test_gait_planning.getCurrentStanceFoot(), -1) << "Current stance foot is wrong"; 
    ASSERT_EQ((int)test_gait_planning.getCurrentStanceFoot(), 0) << "Current stance foot is not initialized correctly";
}

TEST_F(GaitPlanningTest, SetLeftStanceCorrect){
    test_gait_planning.setStanceFoot(-1);
    ASSERT_NE(test_gait_planning.getCurrentStanceFoot(), 1) << "Current stance foot is wrong"; 
    ASSERT_NE(test_gait_planning.getCurrentStanceFoot(), 0) << "Current stance foot is wrong"; 
    ASSERT_EQ(test_gait_planning.getCurrentStanceFoot(), -1) << "Current stance foot is not initialized correctly"; 
}

TEST_F(GaitPlanningTest, AssertErrorWrongInputSetStanceFoot){
    // assert an error is caught when wrong ints are used as input
}

TEST_F(GaitPlanningTest, CorrectSetFootPositions){
    std::array<double, 3> left_foot_pos = {1.2, 1.3, 1.4}; 
    std::array<double, 3> right_foot_pos = {0.1, 0.2, 0.3}; 
    test_gait_planning.setFootPositions(left_foot_pos, right_foot_pos);
    ASSERT_EQ(test_gait_planning.getCurrentLeftFootPos(), left_foot_pos) << "Left foot position is not initialized correctly";
    ASSERT_EQ(test_gait_planning.getCurrentRightFootPos(), right_foot_pos) << "Right foot position is not initialized correctly";
}

TEST_F(GaitPlanningTest, AssertErrorWrongInputSetFootPositions){
    // assert an error is caught when wrong arrays and vectors are used as input
}

TEST_F(GaitPlanningTest, SetGaitTypeCorrect){
    ExoMode mode = ExoMode::Walk;
    test_gait_planning.setGaitType(mode);
    ASSERT_EQ(test_gait_planning.getGaitType(), mode) << "Current stance foot is not initialized correctly";
}

TEST_F(GaitPlanningTest, AssertErrorWrongInputSetGaitType){
    // assert error is caught when input is not an ExoMode
}

TEST_F(GaitPlanningTest, FirstStepTrajectoryGetterTest){
    // test that correct member trajectory is got when double stance foot 
    test_gait_planning.setStanceFoot(0); 
    ASSERT_EQ((int)test_gait_planning.getTrajectory().size(), 20) << "First step trajectory was not returned whilst double stance"; 
}

TEST_F(GaitPlanningTest, FullStepTrajectoryGetterTest){
    // test that correct member trajectory is got when double stance foot 
    test_gait_planning.setStanceFoot(1); 
    ASSERT_EQ((int)test_gait_planning.getTrajectory().size(), 40) << "Full step trajectory was not returned whilst no double stance"; 
}