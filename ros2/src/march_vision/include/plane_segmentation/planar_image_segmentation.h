#pragma once

#include <Eigen/Core>

#include <grid_map_core/GridMap.hpp>
#include <opencv2/core/mat.hpp>

#include "march_vision/segmented_planes_map.h"
#include "march_vision/ransac_segmentation.h"

namespace march_vision {

struct PlanarImageSegmentationParameters {
  /// Size of the sliding window patch used for normal vector calculation and planarity detection
  /// Should be an odd number and at least 3.
  int kernel_size = 3;

  /// [#] Apply opening filter (erosion -> dilation) to planarity detection by this amount of pixels
  int planarity_opening_filter = 0;

  /// [-] Maximum allowed angle between the surface normal and the world-z direction for a patch (converted to dotproduct bound)
  double plane_inclination_threshold = std::cos(30.0 * M_PI / 180.0);

  /// [-] Maximum allowed angle between the surface normal and the world-z direction for a cell (converted to dotproduct bound)
  double local_plane_inclination_threshold = std::cos(35.0 * M_PI / 180.0);

  /// [m] The allowed root-mean-squared deviation from the plane fitted to the patch. Higher -> not planar
  double plane_patch_error_threshold = 0.02;

  /// [#] Labels with less points assigned to them are discarded
  int min_number_points_per_label = 4;

  /// Label kernel connectivity. 4 or 8 (cross or box)
  int connectivity = 4;

  /// Enable RANSAC refinement if the plane is not globally fitting to the assigned points.
  bool include_ransac_refinement = true;

  /// [m] Allowed maximum distance from a point to the plane. If any point violates this, RANSAC is triggered
  double global_plane_fit_distance_error_threshold = 0.025;

  /// [deg] Allowed normal vector deviation for a point w.r.t. the plane normal. If any point violates this, RANSAC is triggered
  double global_plane_fit_angle_error_threshold_degrees = 25.0;
};


class PlanarImageSegmentation {

 public:

  PlanarImageSegmentation(const PlanarImageSegmentationParameters& parameters,
                              const ransac_segmentation::RansacSegmentationParameters& ransac_parameters);
  void runExtraction(const grid_map::GridMap& map, const std::string& layer_height);
  const SegmentedPlanesMap& getSegmentedPlanesMap() const { return m_segmented_planes_map; }
  const cv::Mat& getBinaryLabeledImage() const { return m_binary_image_patch; }
  /** Can be run after extraction for debugging purpose */
  void addSurfaceNormalToMap(grid_map::GridMap& map, const std::string& layer_prefix) const;

 private:

  bool isGloballyPlanar(const Eigen::Vector3d& normal_vector_plane, const Eigen::Vector3d& support_vector_plane,
                        const std::vector<ransac_segmentation::PointWithNormal>& points_with_normal) const;
  bool isWithinInclinationLimit(const Eigen::Vector3d& normal_vector_plane) const;
  std::pair<Eigen::Vector3d, double> computeNormalAndErrorForWindow(const Eigen::MatrixXf& window_data) const;
  bool isLocallyPlanar(const Eigen::Vector3d& localNormal, double mean_squared_error) const;
  int getLinearIndex(int row, int col) const { return row + col * m_map_rows; };
  void computePlaneParametersForLabel(int label, std::vector<ransac_plane_extractor::PointWithNormal>& points_with_normal);
  void refineLabelWithRansac(int label, std::vector<ransac_plane_extractor::PointWithNormal>& points_with_normal);
  void extractPlaneParametersFromLabeledImage();
  void runSegmentation();
  void runSlidingWindowDetector();
  void setToBackground(int label);

  PlanarImageSegmentationParameters m_parameters;
  ransac_segmentation::RansacSegmentationParameters m_ransac_parameters;

  const grid_map::GridMap* m_map;
  std::string m_elevation_layer;
  int m_map_rows;

  std::vector<Eigen::Vector3d> m_surface_normals;
  std::vector<ransac_plane_extractor::PointWithNormal> m_points_with_normal;

  cv::Mat m_binary_image_patch;
  SegmentedPlanesMap m_segmented_planes_map;
};
}  // namespace march_vision
