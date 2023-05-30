//
// Created by Jack Zeng march8 on 30-05-2023
//

#include "state_estimator_mock/state_estimator_mock_node.hpp"
#include "state_estimator_mock/state_estimator_mock.hpp"

StateEstimatorMock::StateEstimatorMock()
    : m_current_shooting_node(0)
{
    
}

void StateEstimatorMock::set_current_shooting_node(int current_shooting_node)
{
    m_current_shooting_node = current_shooting_node;
}