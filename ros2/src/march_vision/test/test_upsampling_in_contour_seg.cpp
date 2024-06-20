#include <gtest/gtest.h>

#include "march_vision/contour_segmentation.h"

TEST(TestUpsampling, upsampleImage) {
  // clang-format off
  cv::Mat M = (cv::Mat_<float>(3, 3) <<
      1, 2, 3,
      4, 5, 6,
      7, 8, 9);
  cv::Mat MoutCheck = (cv::Mat_<float>(7, 7) <<
      1, 1, 2, 2, 2, 3, 3,
      1, 1, 2, 2, 2, 3, 3,
      4, 4, 5, 5, 5, 6, 6,
      4, 4, 5, 5, 5, 6, 6,
      4, 4, 5, 5, 5, 6, 6,
      7, 7, 8, 8, 8, 9, 9,
      7, 7, 8, 8, 8, 9, 9);
  // clang-format on

  const auto Mout = march_vision::contour_segmentation::upSample(M);

  ASSERT_TRUE(std::equal(MoutCheck.begin<float>(), MoutCheck.end<float>(), Mout.begin<float>()));
}