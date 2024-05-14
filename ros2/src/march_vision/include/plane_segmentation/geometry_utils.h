#pragma once

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>

#include "march_vision/plane_segmentation/planar_region.h"

namespace plane_segmentation {

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using CgalPoint2d = K::Point_2;
using CgalCircle2d = K::Circle_2;
using CgalPolygon2d = CGAL::Polygon_2<K>;
using CgalSegment2d = CgalPolygon2d::Segment_2;
using CgalPolygonWithHoles2d = CGAL::Polygon_with_holes_2<K>;
using CgalBbox2d = CGAL::Bbox_2;

inline bool doEdgesIntersect(const CgalSegment2d& line, const CgalPolygon2d& contour) {

  for (auto edgeIt = contour.edges_begin(); edgeIt != contour.edges_end(); ++edgeIt) {
    if (CGAL::do_intersect(line, *edgeIt)) {
      return true;
    }
  }
  return false;
}

inline bool doEdgesIntersect(const CgalSegment2d& line, const CgalPolygonWithHoles2d& parent_shape) {

  if (doEdgesIntersect(line, parent_shape.outer_boundary())) {
    return true;
  } else {
    for (const auto& hole : parent_shape.holes()) {
      if (doEdgesIntersect(line, hole)) {
        return true;
      }
    }
  }
  return false;
}

inline double squaredDistance(const CgalPoint2d& point, const CgalPolygon2d& polygon) {

  double min_dist_squared = std::numeric_limits<double>::max();
  for (auto edgeIt = polygon.edges_begin(); edgeIt != polygon.edges_end(); ++edgeIt) {
    double dist_square = CGAL::squared_distance(point, *edgeIt);
    min_dist_squared = std::min(dist_square, min_dist_squared);
  }
  return min_dist_squared;
}

inline double squaredDistance(const CgalPoint2d& point, const CgalPolygonWithHoles2d& parent_shape) {

  double min_dist_squared = squaredDistance(point, parent_shape.outer_boundary());
  for (const auto& hole : parent_shape.holes()) {
    double dist_square = squaredDistance(point, hole);
    min_dist_squared = std::min(dist_square, min_dist_squared);
  }
  return min_dist_squared;
}

inline double squaredDistance(const CgalPoint2d& point, const CgalCircle2d& circle) {
  auto dx = (point.x() - circle.center().x());
  auto dy = (point.y() - circle.center().y());
  return dx * dx + dy * dy;
}

template <typename T>
double distance(const CgalPoint2d& point, const T& shape) {
  double distance_squared = squaredDistance(point, shape);
  return (distance_squared > 0.0) ? std::sqrt(distance_squared) : 0.0;
}

inline bool isInside(const CgalPoint2d& point, const CgalCircle2d& circle) {
  return squaredDistance(point, circle) <= circle.squared_radius();
}

inline bool isInside(const CgalPoint2d& point, const CgalPolygon2d& polygon) {
  const auto bounded_side = CGAL::bounded_side_2(polygon.begin(), polygon.end(), point);
  return bounded_side == CGAL::ON_BOUNDED_SIDE || bounded_side == CGAL::ON_BOUNDARY;
}

inline bool isInside(const CgalPoint2d& point, const CgalPolygonWithHoles2d& polygon_with_holes) {
  if (isInside(point, polygon_with_holes.outer_boundary())) {
    // Inside the outer contour -> return false if the point is inside any of the holes
    for (const auto& hole : polygon_with_holes.holes()) {
      const auto bounded_side = CGAL::bounded_side_2(hole.begin(), hole.end(), point);
      if (bounded_side == CGAL::ON_BOUNDED_SIDE) {  // The edge of the hole is considered part of the polygon
        return false;
      }
    }
    return true;
  } else {
    return false;
  }
}

inline CgalPoint2d getPointOnLine(const CgalPoint2d& start, const CgalPoint2d& end, double factor) {
  return {factor * (end.x() - start.x()) + start.x(), factor * (end.y() - start.y()) + start.y()};
}

inline CgalPoint2d projectToClosestPoint(const CgalPoint2d& point, const CgalSegment2d& segment) {
  // The segment as a vector, with the source being the origin
  const Eigen::Vector2d source_to_target{segment.target().x() - segment.source().x(), segment.target().y() - segment.source().y()};
  const double source_to_target_distance = source_to_target.norm();
  const Eigen::Vector2d n = source_to_target / source_to_target_distance;

  // Vector from source to the query point
  const Eigen::Vector2d source_to_point{point.x() - segment.source().x(), point.y() - segment.source().y()};

  // Projection to the line, clamped to be between source and target points
  const double coeff = std::min(std::max(0.0, n.dot(source_to_point)), source_to_target_distance);

  return {coeff * n.x() + segment.source().x(), coeff * n.y() + segment.source().y()};
}

inline CgalPoint2d projectToClosestPoint(const CgalPoint2d& point, const CgalPolygon2d& polygon) {

  double min_dist_squared = CGAL::squared_distance(point, *polygon.edges_begin());
  auto closest_edge = polygon.edges_begin();
  for (auto edgeIt = std::next(polygon.edges_begin()); edgeIt != polygon.edges_end(); ++edgeIt) {
    double dist_square = CGAL::squared_distance(point, *edgeIt);
    if (dist_square < min_dist_squared) {
      closest_edge = edgeIt;
      min_dist_squared = dist_square;
    }
  }
  return projectToClosestPoint(point, *closest_edge);
}

inline void transformInPlace(CgalPolygon2d& polygon, const std::function<void(CgalPoint2d&)>& f) {
  for (auto& point : polygon) {
    f(point);
  }
}

inline void transformInPlace(CgalPolygonWithHoles2d& polygon_with_holes, const std::function<void(CgalPoint2d&)>& f) {

  transformInPlace(polygon_with_holes.outer_boundary(), f);
  for (auto& hole : polygon_with_holes.holes()) {
    transformInPlace(hole, f);
  }
}

inline void transformInPlace(BoundaryWithInset& boundary_with_inset, const std::function<void(CgalPoint2d&)>& f) {

  transformInPlace(boundary_with_inset.boundary, f);
  for (auto& inset : boundary_with_inset.insets) {
    transformInPlace(inset, f);
  }
}

}  // namespace plane_segmentation
