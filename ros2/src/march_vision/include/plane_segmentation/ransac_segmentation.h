#pragma once

#include <vector>

#include <Eigen/Core>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/property_map.h>

namespace plane_segmentation {

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

struct RansacSegmentationParameters {
  /// Set probability to miss the largest primitive at each iteration.
  double probability = 0.01;
  /// Detect shapes with at least N points.
  double min_points = 4;
  /// [m] Set maximum Euclidean distance between a point and a shape.
  double epsilon = 0.025;
  /// Set maximum Euclidean distance between points to be clustered. Two points are connected if separated by a distance of at most
  /// 2*sqrt(2)*cluster_epsilon = 2.828 * cluster_epsilon
  double cluster_epsilon = 0.08;
  /// [deg] Set maximum normal deviation between cluster surface_normal and point normal.
  double normal_threshold = 25.0;
};

class RansacSegmentation {
 public:
  RansacSegmentation(const RansacSegmentationParameters& parameters);

  void setParameters(const RansacSegmentationParameters& parameters);

  void detectPlanes(std::vector<PointWithNormal>& points_with_normal);

  /// Return {plane normal, support vector} for the detected shape
  static std::pair<Eigen::Vector3d, Eigen::Vector3d> getPlaneParameters(Shape* shapePtr);

  /// Returns an iterator range. Data is still in the ransac_object
  EfficientRansac::Shape_range getDetectedPlanes() const { return m_ransac.shapes(); };

  /// Returns an iterator range. Data is still in the ransac_object
  EfficientRansac::Point_index_range getUnassignedPointIndices() { return m_ransac.indices_of_unassigned_points(); }

 private:
  EfficientRansac m_ransac;
  EfficientRansac::Parameters m_cgal_ransac_parameters;
};

}  // namespace plane_segmentation
