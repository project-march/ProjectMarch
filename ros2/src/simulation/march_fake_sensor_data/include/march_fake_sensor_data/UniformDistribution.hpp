// Copyright Project MARCH 2020
#pragma once

#include <random>

class UniformDistribution {
private:
    // Variables that are required for random number generation.
    std::random_device random_device;
    std::mt19937 generator;
    std::uniform_int_distribution<int> distribution;

public:
    UniformDistribution(const int start, const int end);

    // Assume the start to be 0 if only the end is specified.
    explicit UniformDistribution(const int end)
        : UniformDistribution { 0, end } {};

    // While the constructor does also create the random device and the random
    // generator, there should also be a possibility to change the range of the
    // generator without replacing it fully. In order to ensure that the pseudo
    // random numbers have random characteristics, it is necessary that the
    // random device and generator are kept the same.
    void set_range(const int start, const int end);

    // Get a uniformly distributed pseudo random number in the specified range.
    int get_random_number();
};
