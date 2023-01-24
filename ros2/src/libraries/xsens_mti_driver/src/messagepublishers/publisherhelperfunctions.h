// BSD 3-Clause License
//
// Copyright (c) 2021, BlueSpace.ai, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef PUBLISHER_HELPER_FUNCTION_H
#define PUBLISHER_HELPER_FUNCTION_H

#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>

class PublisherHelperFunctions
{
public:
    PublisherHelperFunctions(/* args */);
    ~PublisherHelperFunctions();

    void variance_from_stddev_param(std::string param, double *variance_out, rclcpp::Node& node_handle)
    {
        std::vector<double> stddev;
        if (node_handle.get_parameter(param, stddev))
        {
            if (stddev.size() == 3)
            {
                auto squared = [](double x) { return x * x; };
                std::transform(stddev.begin(), stddev.end(), variance_out, squared);
            }
            else
            {
                RCLCPP_WARN(node_handle.get_logger(), "Wrong size of param: %s, must be of size 3", param.c_str());
            }
        }
        else
        {
            memset(variance_out, 0, 3 * sizeof(double));
        }
    }

};

PublisherHelperFunctions::PublisherHelperFunctions(/* args */)
{
}

PublisherHelperFunctions::~PublisherHelperFunctions()
{
}

#endif

