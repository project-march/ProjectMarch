#pragma once

// #include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <visualization_msgs/msg/marker.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <plane_segmentation/planar_region.h>

namespace plane_segmentation {

geometry_msgs::msg::PolygonStamped to3dRosPolygon(const CgalPolygon2d& polygon, const Eigen::Isometry3d& transform_plane_to_world,
                                             const std_msgs::msg::Header& header);

std::vector<geometry_msgs::msg::PolygonStamped> to3dRosPolygon(const CgalPolygonWithHoles2d& polygon_with_holes,
                                                          const Eigen::Isometry3d& transform_plane_to_world, const std_msgs::msg::Header& header);

visualization_msgs::msg::MarkerArray convertBoundariesToRosMarkers(const std::vector<PlanarRegion>& planar_regions, const std::string& frameId,
                                                              grid_map::Time time, double line_width);

visualization_msgs::msg::MarkerArray convertInsetsToRosMarkers(const std::vector<PlanarRegion>& planar_regions, const std::string& frameId,
                                                          grid_map::Time time, double line_width);

}  // namespace plane_segmentation