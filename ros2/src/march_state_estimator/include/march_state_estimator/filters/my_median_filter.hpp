/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef MARCH_FILTERS__MY_MEDIAN_FILTER_HPP_
#define MARCH_FILTERS__MY_MEDIAN_FILTER_HPP_

#include "filters/median.hpp"

class MyMedianFilter : public filters::MedianFilter <double>
{
public:
  typedef std::shared_ptr<MyMedianFilter> SharedPtr;

  bool configure(int number_of_observation) {
    number_of_observations_ = number_of_observation;
    data_storage_.reset(new filters::RealtimeCircularBuffer<double>(number_of_observations_, temp));
    temp_storage_.resize(number_of_observations_);
    return true;
  }
};

#endif  // MARCH_FILTERS__MY_MEAN_FILTER_HPP_