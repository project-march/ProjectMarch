#include "march_gait_planning/gait_planning_node.hpp"
#include "gtest/gtest.h"

class GaitPlanningNodeTest : public testing::Test {
    protected: 
    GaitPlanningNode test_gait_planning_node = GaitPlanningNode();
};

TEST_F(GaitPlanningNodeTest, CurrentStateCallbackTest){
    // Test the message type and content 
}

TEST_F(GaitPlanningNodeTest, TimerCallbackTest){
    // Test dt of timer 
}
TEST_F(GaitPlanningNodeTest, CurrentFeetPositionsCallbackTest){
    // Test message type and content 
}

TEST_F(GaitPlanningNodeTest, ResponseStanceLegCallback){
    //  Test message type and content
}

TEST_F(GaitPlanningNodeTest, SendRequestTest){
    // Test if sending was correct, check if m_stance leg request and m response received are edited
}

// TEST_F(GaitPlanningNodeTest, SetFootPositionsMessageTest){
//     test_gait_planning.setFootPositionsMessage(0.1, 0.2, 0.3, 0.4, 0.5, 0.6); 
//     test_gait_planning.footPositionsPublish(); 
//     ASSERT_EQ(test_gait_planning.m_desired_footpositions_msg->left_foot_position, ({0.1, 0.2, 0.3})); 
//     ASSERT_EQ(test_gait_planning.m_desired_footpositions_msg->right_foot_position, ({0.4, 0.5, 0.6})); 
// }

// TEST_F(GaitPlanningNodeTest, FootPositionsPublishStandTest){
//     test_gait_planning.m_gait_planning.setGaitType(exoMode::Stand);
//     test_gait_planning.footPositionsPublish(); 
//     ASSERT_EQ(test_gait_planning.m_desired_footpositions_msg->left_foot_position, ({0.0, 0.16, -0.802})); 
//     ASSERT_EQ(test_gait_planning.m_desired_footpositions_msg->right_foot_position, ({0.0, -0.16, -0.802})); 
//     ASSERT_TRUE(test_gait_planning.m_current_trajectory.empty())
// }

// TEST_F(GaitPlanningNodeTest, FootPositionsPublishBootUpTest){
//     test_gait_planning.m_gait_planning.setGaitType(exoMode::BootUp); 
//     test_gait_planning.footPositionsPublish(); 
//     ASSERT_TRUE(test_gait_planning.m_desired_footpositions_msg.empty()); 
// }

// TEST_F(GaitPlanningNodeTest, FootPositionsPublishFirstStepTest){
//     test_gait_planning.m_gait_planning.setStanceFoot(0); 
//     test_gait_planning.m_gait_planning.setGaitType(exoMode::Walk); 
//     test_gait_planning.footPositionsPublish(); 
//     ASSERT_EQ(test_gait_planning.m_desired_footpositions_msg->left_foot_position, ({}))
// }



