#include <gtest/gtest.h>

TEST(march_state_estimator, robot_description_node)
{
    // EXPECT_EQ(2 + 2, 1);
    ASSERT_EQ(4, 2 + 2);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}