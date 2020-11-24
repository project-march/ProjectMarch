// Copyright 2020 Project March.

#include "march_fake_sensor_data/UniformDistribution.hpp"

/**
 * @file UniformDistribution.cpp
 * @brief Provide a wrapper around the random functions in the standard library.
 * @details The wrapper makes sure that the same distribution will be used, unless
 * the range of the distribution is changed. In that case, a new distribution is
 * created. This is necessary to ensure that the generated pseudo random distribution
 * has pseude random properties.
 */

/**
 * @brief Create a random uniform distribution between start and end
 * @param start The lower bound (inclusive)
 * @param end The upper bound (inclusive)
 * @return An object that can generate random distributions.
 */
UniformDistribution::UniformDistribution(const int start, const int end):
    generator(random_device())
{
    set_range(start, end);
}

/**
 * @brief Set the range of an existing distribution to a new range.
 * @param start The lower bound (inclusive)
 * @param end The upper bound (inclusive)
 * @throws BadUniformDistribution If the end is smaller than the start.
 */
void UniformDistribution::set_range(const int start, const int end)
{
    // Invariant: check that the end is larger than the start.
    if (end < start) throw BadUniformDistribution {};

    std::uniform_int_distribution<int> new_distribution(start, end);
    distribution = new_distribution;
}

/**
 * @brief Get a random number from the uniform distribution.
 */
int UniformDistribution::get_random_number()
{
    return distribution(generator);
}
