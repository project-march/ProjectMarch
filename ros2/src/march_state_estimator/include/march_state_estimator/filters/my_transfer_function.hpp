/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef MARCH_FILTERS__MY_TRANSFER_FUNCTION_HPP_
#define MARCH_FILTERS__MY_TRANSFER_FUNCTION_HPP_

#include "filters/transfer_function.hpp"

class MyTransferFunction : public filters::SingleChannelTransferFunctionFilter<double>
{
public:
  typedef std::shared_ptr<MyTransferFunction> SharedPtr;

  bool configure(
    const std::vector<double>& output_coefficients, 
    const std::vector<double>& input_coefficients) {
    a_ = output_coefficients;
    b_ = input_coefficients;

    input_buffer_.reset(new filters::RealtimeCircularBuffer<double>(b_.size() - 1, temp_));
    output_buffer_.reset(new filters::RealtimeCircularBuffer<double>(a_.size() - 1, temp_));

    // Normalize the coeffs by a[0].
    if (a_[0] != 1.) {
    for (size_t i = 0; i < b_.size(); i++) {
        b_[i] = (b_[i] / a_[0]);
    }
    for (size_t i = 1; i < a_.size(); i++) {
        a_[i] = (a_[i] / a_[0]);
    }
    a_[0] = 1.;
    }

    return true;
  }
};

#endif  // MARCH_FILTERS__MY_TRANSFER_FUNCTION_HPP_