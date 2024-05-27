#ifndef __clang_analyzer__
// NOLINTBEGIN

#include "march_mpc_solver/mpc_solver.hpp"
#include <array>
#include <gtest/gtest.h>
#include <memory>

class MpcSolverTest : public testing::Test {
protected:
    std::unique_ptr<MpcSolver> mpc_solver;

private:
    void SetUp() override
    {
        mpc_solver = std::make_unique<MpcSolver>();
    }
};

TEST_F(MpcSolverTest, setterGetterTest)
{
    // NX = 0;
    std::array<double, NX> test_state = { 0, 2, 4, 1, 3, 5, 6, 8, 7, 9, 0, 0 };

    this->mpc_solver->set_current_com(0, 1, 2, 3);
    this->mpc_solver->set_current_zmp(4, 5);
    this->mpc_solver->set_current_foot(6, 7);
    this->mpc_solver->set_previous_foot(8, 9);

    this->mpc_solver->set_current_state();
    ASSERT_EQ(this->mpc_solver->get_state(), test_state);
}

TEST_F(MpcSolverTest, MpcIsSolvableTest)
{
    // With this, we can also check if our initial conditions allow for a solvable start
    this->mpc_solver->initialize_mpc_params();

    this->mpc_solver->set_current_com(0.0, 0.18, 0, 0);
    this->mpc_solver->set_current_zmp(0, 0.2);
    this->mpc_solver->set_current_foot(0, 0.2);
    this->mpc_solver->set_previous_foot(0, 0);

    this->mpc_solver->set_current_state();

    int solving_status = this->mpc_solver->solve_step();
    // If we get 0, We can solve at least.
    ASSERT_EQ(solving_status, 0);
}

// NOLINTEND
#endif
