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

#include <march_ik_solver/ik_solver.hpp>

class IkSolverTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        m_ik_solver = std::make_unique<IKSolver>();
    }

    std::unique_ptr<IKSolver> m_ik_solver;
};

TEST_F(IkSolverTest, test)
{
  ASSERT_EQ(2, 1+1);
}

// NOLINTEND
#endif  // __clang_analyzer__