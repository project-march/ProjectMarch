#include "gtest/gtest.h"
#include "march_gait_planning/gait_planning_joint_angles_node.hpp"

class GaitPlanningAnglesNodeTest : public testing::Test {
    protected: 
    void SetUp() override {
        m_test_gait_planning_angles = std::make_unique<GaitPlanningAnglesNode>(); 
    }
}; 