#ifndef IK_SOLVER_NODE_H
#define IK_SOLVER_NODE_H

#include "ik_solver/ik_solver.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cstdio>

class IkSolverNode : public rclcpp::Node {
public:
    IkSolverNode();

private:
    IkSolver m_ik_solver;
};

#endif