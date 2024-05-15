#pragma once

#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <march_vision/plane_segmentation/planar_region.h>

namespace plane_segmentation {

geometry_msgs::PolygonStamped to3dRosPolygon(const CgalPolygon2d& polygon, const Eigen::Isometry3d& transform_plane_to_world,
                                             const std_msgs::Header& header);

std::vector<geometry_msgs::PolygonStamped> to3dRosPolygon(const CgalPolygonWithHoles2d& polygon_with_holes,
                                                          const Eigen::Isometry3d& transform_plane_to_world, const std_msgs::Header& header);

visualization_msgs::MarkerArray convertBoundariesToRosMarkers(const std::vector<PlanarRegion>& planar_regions, const std::string& frameId,
                                                              grid_map::Time time, double line_width);

visualization_msgs::MarkerArray convertInsetsToRosMarkers(const std::vector<PlanarRegion>& planar_regions, const std::string& frameId,
                                                          grid_map::Time time, double line_width);

}  // namespace plane_segmentation