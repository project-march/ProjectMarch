#pragma once

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>

namespace march_vision {

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using CgalPoint2d = K::Point_2;
using CgalCircle2d = K::Circle_2;
using CgalPolygon2d = CGAL::Polygon_2<K>;
using CgalSegment2d = CgalPolygon2d::Segment_2;
using CgalPolygonWithHoles2d = CGAL::Polygon_with_holes_2<K>;
using CgalBbox2d = CGAL::Bbox_2;

CgalPolygon2d createRegularPolygon(const CgalPoint2d& center, double radius, int numberOfVertices);

CgalPolygon2d growConvexPolygonInsideShape(const CgalPolygon2d& parentShape, CgalPoint2d center, int numberOfVertices, double growthFactor);

CgalPolygon2d growConvexPolygonInsideShape(const CgalPolygonWithHoles2d& parentShape, CgalPoint2d center, int numberOfVertices,
                                           double growthFactor);

void updateMean(CgalPoint2d& mean, const CgalPoint2d& oldValue, const CgalPoint2d& updatedValue, int N);

}  // namespace march_vision
