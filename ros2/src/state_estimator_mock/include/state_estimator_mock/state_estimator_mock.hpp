//
// Created by Jack Zeng on 30-05-2023.
//

#ifndef STATE_ESTIMATOR_MOCK_HPP
#define STATE_ESTIMATOR_MOCK_HPP
#include "std_msgs/msg/float32.hpp"

class StateEstimatorMock {
public:
    StateEstimatorMock();
    void set_current_shooting_node(int);
private:  
    int m_current_shooting_node;
};

#endif // STATE_ESTIMATOR_MOCK_HPP