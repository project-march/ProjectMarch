//
// Created by AlexanderJamesBecoy on 2023-01-10.
//

#ifndef __clang_analyzer__
// NOLINTBEGIN
// Copyright 2023 Project March.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include <march_ik_solver/task.hpp>

class TaskTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        m_task = std::make_unique<Task>();
    }

    std::unique_ptr<Task> m_task;
};

TEST_F(TaskTest, test)
{
  ASSERT_EQ(2, 1+1);
}

// NOLINTEND
#endif  // __clang_analyzer__