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
    EXPECT_EQ(test_gait_planning.getCurrentStanceFoot(), 0) << "Current stance foot is not initialized correctly";
}

TEST_F(GaitPlanningTest, AssertErrorWrongInputSetStanceFoot){
    // assert an error is caught when doubles, floats or strings are used as input
}

TEST_F(GaitPlanningTest, CorrectSetFootPositions){
    std::array<double, 3> left_foot_pos = {1.2, 1.3, 1.4}; 
    std::array<double, 3> right_foot_pos = {0.1, 0.2, 0.3}; 
    test_gait_planning.setFootPositions(left_foot_pos, right_foot_pos);
    EXPECT_EQ(test_gait_planning.getCurrentLeftFootPos(), left_foot_pos) << "Left foot position is not initialized correctly";
    EXPECT_EQ(test_gait_planning.getCurrentRightFootPos(), right_foot_pos) << "Right foot position is not initialized correctly";
}

TEST_F(GaitPlanningTest, AssertErrorWrongInputSetFootPositions){
    // assert an error is caught when wrong arrays and vectors are used as input
}

TEST_F(GaitPlanningTest, SetGaitTypeCorrect){
    exoState state = exoState::Walk;
    test_gait_planning.setGaitType(state);
    EXPECT_EQ(test_gait_planning.getGaitType(), state) << "Current stance foot is not initialized correctly";
}

TEST_F(GaitPlanningTest, AssertErrorWrongInputSetGaitType){
    // assert error is caught when input is not an exoState
}

TEST_F(GaitPlanningTest, BezierFileCreated){
    // test if bezier file exists and is not empty
}

TEST_F(GaitPlanningTest, BezierTimeStepsCorrect){
    // assert the bezier files have lengths of 20 and 40 
}

// TEST all getters 

int main(int argc, char** argv){
    testing::InitGoogleTest(&argc, argv); 
    return RUN_ALL_TESTS(); 
}