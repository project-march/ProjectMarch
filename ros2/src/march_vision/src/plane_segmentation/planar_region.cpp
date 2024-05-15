#include "march_vision/plane_segmentation/planar_region.h"

namespace plane_segmentation {

// TODO: Remove if we are not going to use global frame
Eigen::Isometry3d getTransformLocalToGlobal(const NormalAndPosition& normal_and_position) {

  const Eigen::Vector3d z_axis_in_global = normal_and_position.normal.normalized();
  const auto& position_in_global = normal_and_position.position;

  // Cross with any vector that is not equal to surfaceNormal
  Eigen::Vector3d y_axis_in_global = z_axis_in_global.cross(Eigen::Vector3d::UnitX());
  {  // Normalize the yAxis. Need to pick a different direction if z happened to intersect with unitX
    const auto y_squared_norm = y_axis_in_global.squaredNorm();
    const double cross_tolerance = 1e-3;
    if (y_squared_norm > cross_tolerance) {
      y_axis_in_global /= std::sqrt(y_squared_norm);
    } else {
      // normal was almost equal to unitX. Pick the y-axis in a different way (approximately equal to unitY):
      y_axis_in_global = z_axis_in_global.cross(Eigen::Vector3d::UnitY().cross(z_axis_in_global)).normalized();
    }
  }

  Eigen::Isometry3d transform;
  transform.linear().col(0) = y_axis_in_global.cross(z_axis_in_global);
  transform.linear().col(1) = y_axis_in_global;
  transform.linear().col(2) = z_axis_in_global;
  transform.translation() = position_in_global;

  return transform;
}

CgalPoint2d projectToPlaneAlongGravity(const CgalPoint2d& worldFrameXY, const Eigen::Isometry3d& transform_plane_to_world) {

  const auto& xAxis = transform_plane_to_world.linear().col(0);
  const auto& yAxis = transform_plane_to_world.linear().col(1);
  const auto& surface_normal_in_world = transform_plane_to_world.linear().col(2);
  const auto& plane_origin_in_world = transform_plane_to_world.translation();

  // Horizontal difference
  const double dx = worldFrameXY.x() - plane_origin_in_world.x();
  const double dy = worldFrameXY.y() - plane_origin_in_world.y();

  // Vertical difference
  // solve surface_normal_in_world.dot(projectedPosition - plane_origin_in_world) = 0
  // with projectPosition XY = worldFrameXY;
  Eigen::Vector3d planeOriginToProjectedPointInWorld(
      dx, dy, (-dx * surface_normal_in_world.x() - dy * surface_normal_in_world.y()) / surface_normal_in_world.z());

  // Project XY coordinates to the plane frame
  return {xAxis.dot(planeOriginToProjectedPointInWorld), yAxis.dot(planeOriginToProjectedPointInWorld)};
}

Eigen::Vector3d positionInWorldFrameFromPosition2dInPlane(const CgalPoint2d& planeXY, const Eigen::Isometry3d& transform_plane_to_world) {
  // Compute transform given that z in plane = 0.0
  Eigen::Vector3d point_in_world = transform_plane_to_world.translation() + planeXY.x() * transform_plane_to_world.linear().col(0) +
                                 planeXY.y() * transform_plane_to_world.linear().col(1);
  return point_in_world;
}

}  // namespace plane_segmentation