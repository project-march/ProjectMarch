#pragma once

#include <Eigen/Core>
#include <opencv2/core/mat.hpp>

#include "plane_segmentation/planar_region.h"

namespace plane_segmentation {

struct SegmentedPlanesMap {
  /// Unordered collection of all labels and corresponding plane parameters
  std::vector<std::pair<int, NormalAndPosition>> label_plane_parameters;

  /// Image with a each pixel being assigned and integer value corresponding to the label. Might contain labels that are not in
  /// labelPlaneParameters. These labels should be seen as background.
  cv::Mat labeled_image;

  /// Size of each pixel [m]
  double resolution;

  /// World X-Y position [m] of the (0, 0) pixel in the image.
  Eigen::Vector2d map_origin;

  /// All label values are smaller than or equal to highestLabel
  int highest_label;
};

}  // namespace plane_segmentation
