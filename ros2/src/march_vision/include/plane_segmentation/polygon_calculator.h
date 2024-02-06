#ifndef POLYGON_CALCULATOR_H
#define POLYGON_CALCULATOR_H

#include <vector>
#include <list>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Alpha_shape_vertex_base_2.h>
#include <CGAL/Alpha_shape_face_base_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/algorithm.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Point_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel  K;
typedef K::FT                                                Alpha_value;   
typedef K::Point_2                                           Point;
typedef K::Segment_2                                         Segment;
typedef CGAL::Alpha_shape_vertex_base_2<K>                   Vb;
typedef CGAL::Alpha_shape_face_base_2<K>                     Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>          Tds;
typedef CGAL::Delaunay_triangulation_2<K,Tds>                Triangulation_2;
typedef CGAL::Alpha_shape_2<Triangulation_2>                 Alpha_shape_2;
typedef Alpha_shape_2::Alpha_shape_edges_iterator            Alpha_shape_edges_iterator;
typedef CGAL::Polygon_2<K>                                   Polygon_2;

struct ConcaveHullPocket
{
    int start_bridge_index;
    int end_bridge_index;
    double max_depth;
    int deepest_vertex_index;
    Point deepest_vertex;
    Point cut_direction;
};

class PolygonCalculator
{
public:
    //PolygonCalculator();
    //~PolygonCalculator();

    Polygon_2 findAlphaShapePolygon(std::list<Point> points_in_plane);

private:
    template <class OutputIterator>
    void findAlphaEdges(const Alpha_shape_2& A, OutputIterator out);
    void recursiveApproximateDecomposition(const Polygon_2& concave_hull_polygon, double depth_threshold, std::vector<Polygon_2>& convex_polygons_to_pack);
    bool isHullConvex(const std::vector<Point>& concave_hull_vertices);
    ConcaveHullPocket findFirstConcaveHullPocket(const std::vector<Point>& concave_hull_vertices);
    void removeAllExclusiveVertices(int startBridgeIndex, int endBridgeIndex, std::vector<Point>& concave_hull_vertices);
    int findClosestIntersectionWithRay(const Point& deepest_vertex, const Point& cut_direction, int end_bridge_index, int start_bridge_index, const std::vector<Point>& concave_hull_vertices, Point& other_vertex_for_cutting);
    void cutConcaveHull(const Point& deepest_vertex, const Point& cut_direction, int end_bridge_index, int start_bridge_index, std::vector<Point>& concave_hull_vertices);
    double angleBetween(const Point& p1, const Point& p2, const Point& p3);
    Point calculateCutDirection(const Point& p1, const Point& p2, const Point& p3);
    double calculateMaxDepth(const std::vector<Point>& concave_hull_vertices, int index, const Point& cut_direction);
    double distanceToSegment(const Point& p, const Point& p1, const Point& p2);
    Point calculateIntersectionWithRay(const Point& ray_origin, const Point& ray_direction, const Point& edge_start, const Point& edge_end);
    double squareDistanceBetweenPoints(const Point& p1, const Point& p2);    
};

#endif // POLYGON_CALCULATOR_H
