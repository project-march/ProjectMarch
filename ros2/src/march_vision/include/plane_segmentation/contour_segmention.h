#pragma once

#include <opencv2/core/mat.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/property_map.h>

#include "march_vision/planar_region.h"
#include "march_vision/segmented_planes_map.h"

namespace march_vision {

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point3D = Kernel::Point_3;
using Vector3D = Kernel::Vector_3;
using PointWithNormal = std::pair<Kernel::Point_3, Kernel::Vector_3>;
using PwnVector = std::vector<PointWithNormal>;

using PointMap = CGAL::First_of_pair_property_map<PointWithNormal>;
using NormalMap = CGAL::Second_of_pair_property_map<PointWithNormal>;
using Traits = CGAL::Shape_detection::Efficient_RANSAC_traits<Kernel, PwnVector, PointMap, NormalMap>;
using EfficientRansac = CGAL::Shape_detection::Efficient_RANSAC<Traits>;
using Plane = CGAL::Shape_detection::Plane<Traits>;
using Shape = CGAL::Shape_detection::Shape_base<Traits>;

struct ContourSegmentationParameters {
  /// Size of the kernel creating the boundary offset. In number of (sub) pixels.
  int margin_size = 1;
};

/**
 * Extracts the contours in map resolution, but with the x and y axis flipped.
 * This way all contours are in counter clockwise direction.
 */
class ContourSegmentation {
 public:
  ContourSegmentation(const ContourSegmentationParameters& parameters);

  std::vector<PlanarRegion> extractPlanarRegions(const SegmentedPlanesMap& segmented_planes_map);

 private:
  ContourSegmentationParameters m_contour_parameters;
  cv::Mat m_inset_kernel;
  cv::Mat m_margin_kernel;

  // Memory to reuse between calls
  cv::Mat m_binary_image;
};

/// Modifies the image in-place!
std::vector<BoundaryWithInset> extractBoundaryAndInset(cv::Mat& binary_image, const cv::Mat& erosion_kernel);
std::vector<CgalPolygonWithHoles2d> extractPolygonsFromBinaryImage(const cv::Mat& binary_image);
CgalPolygon2d cgalPolygonFromOpenCv(const std::vector<cv::Point>& openCV_polygon);

// TOOD: Keep this or remove it?
CgalPoint2d pixelToWorldFrame(const CgalPoint2d& pixelspace_cgal_point2d, double resolution, const Eigen::Vector2d& map_offset);

/**
 * Upsamples an image such that all pixels are turned into 9 pixels, with the original pixel in the middle.
 * Around the edges, each pixel is only upsamples in the inward direction.
 *
 * @param image : source image
 * @return upsamples image
 */
cv::Mat upSample(const cv::Mat& image);


/**
 * Upsamples a segmented terrain such that the resulting terrain has 1/3 of the input resolution. (Each cell is split into 9 cells)
 * This specific upsampling ratio makes it possible to keep labels in their exact original location in world frame.
 *
 * @param map_in : source terrain
 * @return upsampled terrain
 */
SegmentedPlanesMap upSample(const SegmentedPlanesMap& map_in);

}  // namespace march_vision

