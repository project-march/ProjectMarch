#include "plane_segmentation/visualization_utils.h"

#include <geometry_msgs/Point32.h>

namespace plane_segmentation {

// Need to convert 2D polygon to 3D polygon for visualization within rviz2 environment
geometry_msgs::PolygonStamped to3dRosPolygon(const CgalPolygon2d& polygon, const Eigen::Isometry3d& transform_plane_to_world,
                                             const std_msgs::Header& header) {

  geometry_msgs::PolygonStamped polygon3d;
  polygon3d.header = header;
  polygon3d.polygon.points.reserve(polygon.size());
  for (const auto& point : polygon) {
    geometry_msgs::Point32 point_ros;
    const auto point_in_world = positionInWorldFrameFromPosition2dInPlane(point, transform_plane_to_world);
    point_ros.x = static_cast<float>(point_in_world.x());
    point_ros.y = static_cast<float>(point_in_world.y());
    point_ros.z = static_cast<float>(point_in_world.z());
    polygon3d.polygon.points.push_back(point_ros);
  }
  return polygon3d;
}

std::vector<geometry_msgs::PolygonStamped> to3dRosPolygon(const CgalPolygonWithHoles2d& polygon_with_holes,
                                                          const Eigen::Isometry3d& transform_plane_to_world, const std_msgs::Header& header) {

  std::vector<geometry_msgs::PolygonStamped> polygons;
  polygons.reserve(polygon_with_holes.number_of_holes() + 1);
  polygons.emplace_back(to3dRosPolygon(polygon_with_holes.outer_boundary(), transform_plane_to_world, header));

  for (const auto& hole : polygon_with_holes.holes()) {
    polygons.emplace_back(to3dRosPolygon(hole, transform_plane_to_world, header));
  }
  return polygons;
}

namespace {  // Helper functions for convertBoundariesToRosMarkers and convertInsetsToRosMarkers

std_msgs::ColorRGBA getColor(int id, float alpha = 1.0) {
  constexpr int num_colors = 7;
  using RGB = std::array<float, 3>;
  static const std::array<std::array<float, 3>, num_colors> color_map{
    RGB{0.0000F, 0.4470F, 0.7410F},
    RGB{0.8500F, 0.3250F, 0.0980F},
    RGB{0.9290F, 0.6940F, 0.1250F},
    RGB{0.4940F, 0.1840F, 0.5560F},
    RGB{0.4660F, 0.6740F, 0.1880F},
    RGB{0.6350F, 0.0780F, 0.1840F},
    RGB{0.2500F, 0.2500F, 0.2500F}
  };

  std_msgs::ColorRGBA color_msg;
  const auto& rgb = color_map[id % num_colors];
  color_msg.r = rgb[0];
  color_msg.g = rgb[1];
  color_msg.b = rgb[2];
  color_msg.a = alpha;
  return color_msg;
}

visualization_msgs::Marker to3dRosMarker(const CgalPolygon2d& polygon, const Eigen::Isometry3d& transform_plane_to_world,
                                         const std_msgs::Header& header, const std_msgs::ColorRGBA& color, int id, double line_width) {

  visualization_msgs::Marker line;
  line.id = id;
  line.header = header;
  line.type = visualization_msgs::Marker::LINE_STRIP;
  line.scale.x = line_width;
  line.color = color;
  if (!polygon.is_empty()) {
    line.points.reserve(polygon.size() + 1);

    for (const auto& point : polygon) {
      const auto point_in_world = positionInWorldFrameFromPosition2dInPlane(point, transform_plane_to_world);
      geometry_msgs::Point point_ros;
      point_ros.x = point_in_world.x();
      point_ros.y = point_in_world.y();
      point_ros.z = point_in_world.z();
      line.points.push_back(point_ros);
    }
    // repeat the first point to close to polygon
    const auto point_in_world = positionInWorldFrameFromPosition2dInPlane(polygon.vertex(0), transform_plane_to_world);
    geometry_msgs::Point point_ros;
    point_ros.x = point_in_world.x();
    point_ros.y = point_in_world.y();
    point_ros.z = point_in_world.z();
    line.points.push_back(point_ros);
  }
  line.pose.orientation.w = 1.0;
  line.pose.orientation.x = 0.0;
  line.pose.orientation.y = 0.0;
  line.pose.orientation.z = 0.0;
  return line;
}

visualization_msgs::MarkerArray to3dRosMarker(const CgalPolygonWithHoles2d& polygon_with_holes,
                                              const Eigen::Isometry3d& transform_plane_to_world, const std_msgs::Header& header,
                                              const std_msgs::ColorRGBA& color, int id, double line_width) {

  visualization_msgs::MarkerArray polygons;

  polygons.markers.reserve(polygon_with_holes.number_of_holes() + 1);
  polygons.markers.emplace_back(to3dRosMarker(polygon_with_holes.outer_boundary(), transform_plane_to_world, header, color, id, line_width));
  ++id;

  for (const auto& hole : polygon_with_holes.holes()) {
    polygons.markers.emplace_back(to3dRosMarker(hole, transform_plane_to_world, header, color, id, line_width));
    ++id;
  }
  return polygons;
}
}  // namespace

// Using markers as viz equivalent to polygon outlines
visualization_msgs::MarkerArray convertBoundariesToRosMarkers(const std::vector<PlanarRegion>& planar_regions, const std::string& frameId,
                                                              grid_map::Time time, double line_width) {

  std_msgs::Header header;
  header.stamp.fromNSec(time);
  header.frame_id = frameId;

  visualization_msgs::Marker delete_marker;
  delete_marker.action = visualization_msgs::Marker::DELETEALL;

  visualization_msgs::MarkerArray polygon_buffer;
  polygon_buffer.markers.reserve(planar_regions.size() + 1);  // lower bound
  polygon_buffer.markers.push_back(delete_marker);
  int colorIdx = 0;

  for (const auto& planar_region : planar_regions) {

    const auto color = getColor(colorIdx++);
    int label = polygon_buffer.markers.size();
    auto boundaries =
        to3dRosMarker(planar_region.boundary_with_inset.boundary, planar_region.transform_plane_to_world, header, color, label, line_width);
    std::move(boundaries.markers.begin(), boundaries.markers.end(), std::back_inserter(polygon_buffer.markers));
  }

  return polygon_buffer;
}

visualization_msgs::MarkerArray convertInsetsToRosMarkers(const std::vector<PlanarRegion>& planar_regions, const std::string& frameId,
                                                          grid_map::Time time, double line_width) {
  std_msgs::Header header;
  header.stamp.fromNSec(time);frameId
  header.frame_id = frameId;

  visualization_msgs::Marker delete_marker;
  delete_marker.action = visualization_msgs::Marker::DELETEALL;

  visualization_msgs::MarkerArray polygon_buffer;
  polygon_buffer.markers.reserve(planar_regions.size() + 1);  // lower bound
  polygon_buffer.markers.push_back(delete_marker);
  int colorIdx = 0;

  for (const auto& planar_region : planar_regions) {
    const auto color = getColor(colorIdx++);
    for (const auto& inset : planar_region.boundary_with_inset.insets) {
      int label = polygon_buffer.markers.size();
      auto insets = to3dRosMarker(inset, planar_region.transform_plane_to_world, header, color, label, line_width);
      std::move(insets.markers.begin(), insets.markers.end(), std::back_inserter(polygon_buffer.markers));
    }
  }
  return polygon_buffer;
}

}  // namespace plane_segmentation