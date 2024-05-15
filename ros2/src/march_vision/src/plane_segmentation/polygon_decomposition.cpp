#include "march_vision/plane_segmentation/polygon_decomposition.h"

#include "march_vision/plane_segmentation/geometry_utils.h"

namespace plane_segmentation {

namespace {

int getCounterClockWiseNeighbour(int i, int last_vertex) {
  return (i > 0) ? i - 1 : last_vertex;
}

int getClockWiseNeighbour(int i, int last_vertex) {
  return (i < last_vertex) ? i + 1 : 0;
}

std::array<CgalPoint2d, 2> getNeighbours(const CgalPolygon2d& polygon, int i) {
  assert(i < polygon.size());
  assert(polygon.size() > 1);
  int last_vertex = static_cast<int>(polygon.size()) - 1;
  int cwNeighbour = getClockWiseNeighbour(i, last_vertex);
  int ccwNeighbour = getCounterClockWiseNeighbour(i, last_vertex);
  return {polygon.vertex(cwNeighbour), polygon.vertex(ccwNeighbour)};
}

std::array<CgalPoint2d, 4> get2ndNeighbours(const CgalPolygon2d& polygon, int i) {
  assert(i < polygon.size());
  assert(polygon.size() > 1);
  int last_vertex = static_cast<int>(polygon.size()) - 1;
  int cwNeighbour1 = getClockWiseNeighbour(i, last_vertex);
  int cwNeighbour2 = getClockWiseNeighbour(cwNeighbour1, last_vertex);
  int ccwNeighbour1 = getCounterClockWiseNeighbour(i, last_vertex);
  int ccwNeighbour2 = getCounterClockWiseNeighbour(ccwNeighbour1, last_vertex);
  return {polygon.vertex(cwNeighbour2), polygon.vertex(cwNeighbour1), polygon.vertex(ccwNeighbour1), polygon.vertex(ccwNeighbour2)};
}

bool remainsConvexWhenMovingPoint(const CgalPolygon2d& polygon, int i, const CgalPoint2d& point) {
  auto second_neighbours = get2ndNeighbours(polygon, i);
  using CgalPolygon2dFixedSize = CGAL::Polygon_2<K, std::array<K::Point_2, 5>>;

  CgalPolygon2dFixedSize sub_polygon;
  sub_polygon.container()[0] = second_neighbours[0];
  sub_polygon.container()[1] = second_neighbours[1];
  sub_polygon.container()[2] = point;
  sub_polygon.container()[3] = second_neighbours[2];
  sub_polygon.container()[4] = second_neighbours[3];
  return sub_polygon.is_convex();
}

bool pointAndNeighboursAreWithinFreeSphere(const std::array<CgalPoint2d, 2>& neighbours, const CgalPoint2d& point,
                                           const CgalCircle2d& circle) {
  return isInside(neighbours[0], circle) && isInside(point, circle) && isInside(neighbours[1], circle);
}

/**
 * Returns {true, 0.0} if either one of the point -> neighbour segments intersects the parent shape
 * Returns {false, minSquareDistance} if none of the segments intersects. minSquareDistance is the minimum square distance between the
 * point and the parent shape
 */
template <typename T>
std::pair<bool, double> doEdgesIntersectAndSquareDistance(const std::array<CgalPoint2d, 2>& neighbours, const CgalPoint2d& point,
                                                          const T& parent_shape);

template <>
std::pair<bool, double> doEdgesIntersectAndSquareDistance(const std::array<CgalPoint2d, 2>& neighbours, const CgalPoint2d& point,
                                                          const CgalPolygon2d& parent_shape) {
  CgalSegment2d segment0{neighbours[0], point};
  CgalSegment2d segment1{neighbours[1], point};

  double min_dist_squared = std::numeric_limits<double>::max();
  for (auto edgeIt = parent_shape.edges_begin(); edgeIt != parent_shape.edges_end(); ++edgeIt) {
    const auto edge = *edgeIt;
    if (CGAL::do_intersect(segment0, edge) || CGAL::do_intersect(segment1, edge)) {
      return {true, 0.0};
    } else {
      min_dist_squared = std::min(min_dist_squared, CGAL::squared_distance(point, edge));
    }
  }

  return {false, min_dist_squared};
}

template <>
std::pair<bool, double> doEdgesIntersectAndSquareDistance(const std::array<CgalPoint2d, 2>& neighbours, const CgalPoint2d& point,
                                                          const CgalPolygonWithHoles2d& parent_shape) {
  const auto intersect_and_distance = doEdgesIntersectAndSquareDistance(neighbours, point, parent_shape.outer_boundary());
  if (intersect_and_distance.first) {
    return {true, 0.0};
  }

  double min_dist_squared = intersect_and_distance.second;
  for (const auto& hole : parent_shape.holes()) {
    const auto hole_intersect_and_distance = doEdgesIntersectAndSquareDistance(neighbours, point, hole);
    if (hole_intersect_and_distance.first) {
      return {true, 0.0};
    } else {
      min_dist_squared = std::min(min_dist_squared, hole_intersect_and_distance.second);
    }
  }

  return {false, min_dist_squared};
}

template <typename T>
bool pointCanBeMoved(const CgalPolygon2d& growth_shape, int i, const CgalPoint2d& candidate_point, CgalCircle2d& free_sphere,
                     const T& parent_shape) {
  if (remainsConvexWhenMovingPoint(growth_shape, i, candidate_point)) {
    auto neighbours = getNeighbours(growth_shape, i);
    if (pointAndNeighboursAreWithinFreeSphere(neighbours, candidate_point, free_sphere)) {
      return true;
    } else {
      // Look for intersections and minimum distances simultaneously
      const auto intersect_and_distance = doEdgesIntersectAndSquareDistance(neighbours, candidate_point, parent_shape);

      if (intersect_and_distance.first) {
        return false;
      } else {
        // Update free sphere around new point
        free_sphere = CgalCircle2d(candidate_point, intersect_and_distance.second);
        return true;
      }
    }
  } else {
    return false;
  }
}

inline std::ostream& operator<<(std::ostream& os, const CgalPolygon2d& p) {
  os << "CgalPolygon2d: \n";
  for (auto it = p.vertices_begin(); it != p.vertices_end(); ++it) {
    os << "\t(" << it->x() << ", " << it->y() << ")\n";
  }
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const CgalPolygonWithHoles2d& p) {
  os << "CgalPolygonWithHoles2d: \n";
  os << "\t" << p.outer_boundary() << "\n";
  os << "\tHoles: \n";
  for (auto it = p.holes_begin(); it != p.holes_end(); ++it) {
    os << "\t\t" << *it << "\n";
  }
  return os;
}

template <typename T>
CgalPolygon2d growConvexPolygonInsideShape_impl(const T& parent_shape, CgalPoint2d center, int number_of_vertices, double growth_factor) {
  const auto center_copy = center;
  constexpr double initial_radius_factor = 0.999;
  constexpr int max_iter = 1000;
  double radius = initial_radius_factor * distance(center, parent_shape);

  CgalPolygon2d growth_shape = createRegularPolygon(center, radius, number_of_vertices);

  if (radius == 0.0) {
    std::cerr << "[growConvexPolygonInsideShape] Zero initial radius. Provide a point with a non-zero offset to the boundary.\n";
    return growth_shape;
  }

  // Cached values per vertex
  std::vector<bool> blocked(number_of_vertices, false);
  std::vector<CgalCircle2d> freeSpheres(number_of_vertices, CgalCircle2d(center, radius * radius));

  int Nblocked = 0;
  int iter = 0;
  while (Nblocked < number_of_vertices && iter < max_iter) {
    for (int i = 0; i < number_of_vertices; i++) {
      if (!blocked[i]) {
        const auto candidate_point = getPointOnLine(center, growth_shape.vertex(i), growth_factor);
        if (pointCanBeMoved(growth_shape, i, candidate_point, freeSpheres[i], parent_shape)) {
          updateMean(center, growth_shape.vertex(i), candidate_point, number_of_vertices);
          growth_shape.vertex(i) = candidate_point;
        } else {
          blocked[i] = true;
          ++Nblocked;
        }
      }
    }
    ++iter;
  }

  if (iter == max_iter) {
    std::cerr << "[growConvexPolygonInsideShape] max iteration in region growing! Debug information: \n";
    std::cerr << "number_of_vertices: " << number_of_vertices << "\n";
    std::cerr << "growth_factor: " << growth_factor << "\n";
    std::cerr << "Center: " << center_copy.x() << ", " << center_copy.y() << "\n";
    std::cerr << parent_shape << "\n";
  }
  return growth_shape;
}

}  // namespace

CgalPolygon2d createRegularPolygon(const CgalPoint2d& center, double radius, int number_of_vertices) {
  assert(number_of_vertices > 2);
  CgalPolygon2d polygon;
  polygon.container().reserve(number_of_vertices);
  double angle = (2. * M_PI) / number_of_vertices;
  for (int i = 0; i < number_of_vertices; ++i) {
    double phi = i * angle;
    double px = radius * std::cos(phi) + center.x();
    double py = radius * std::sin(phi) + center.y();
    // Counter clockwise
    polygon.push_back({px, py});
  }
  return polygon;
}

CgalPolygon2d growConvexPolygonInsideShape(const CgalPolygon2d& parent_shape, CgalPoint2d center, int number_of_vertices,
                                           double growth_factor) {
  return growConvexPolygonInsideShape_impl(parent_shape, center, number_of_vertices, growth_factor);
}

CgalPolygon2d growConvexPolygonInsideShape(const CgalPolygonWithHoles2d& parent_shape, CgalPoint2d center, int number_of_vertices,
                                           double growth_factor) {
  return growConvexPolygonInsideShape_impl(parent_shape, center, number_of_vertices, growth_factor);
}

void updateMean(CgalPoint2d& mean, const CgalPoint2d& old_value, const CgalPoint2d& updated_value, int N) {
  // old_mean = 1/N * ( others + old_value); -> others = N*old_mean - old_value
  // new_mean = 1/N * ( others + new_value); -> new_mean = old_mean - 1/N * old_value + 1/N * updated_value
  mean += 1.0 / N * (updated_value - old_value);
}

}  // namespace plane_segmentation
