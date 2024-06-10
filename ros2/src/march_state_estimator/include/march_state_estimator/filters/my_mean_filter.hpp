/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef MARCH_FILTERS__MY_MEAN_FILTER_HPP_
#define MARCH_FILTERS__MY_MEAN_FILTER_HPP_

#include "filters/mean.hpp"

class MyMeanFilter : public filters::MeanFilter <double>
{
public:
  typedef std::shared_ptr<MyMeanFilter> SharedPtr;

  bool configure(int number_of_observation) {
    number_of_observations_ = number_of_observation;
    data_storage_.reset(new filters::RealtimeCircularBuffer<double>(number_of_observations_, temp_));
    return true;
  }
};

#endif  // MARCH_FILTERS__MY_MEAN_FILTER_HPP_