// Copyright 2019 Project March.
#ifndef MARCH_SAFETY_SAFETY_TYPE_H
#define MARCH_SAFETY_SAFETY_TYPE_H
#include "rclcpp/rclcpp.hpp"

class SafetyType {
public:
    virtual void update() = 0;
};

#endif // MARCH_SAFETY_SAFETY_TYPE_H
