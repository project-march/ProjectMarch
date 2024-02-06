#include <cmath>
#include <iostream>
#include <vector>

#include <plane_segmentation/polygon_calculator.h>

double ALPHA_VALUE = 0.2;
// TODO: Find best value
double DECOMPOSITION_DEPTH_VARIABLE;

// TODO: Don't use CGAL because polygons and points data structures are shit; Rewrite the alpha shape functions?..
// TODO: Change the polygon output to our polygon data structure.

Polygon_2 PolygonCalculator::findAlphaShapePolygon(std::list<Point> points_in_plane){

    // Delaunay triangulation with chosen alpha value
    Alpha_shape_2 A(points_in_plane.begin(), points_in_plane.end(),
                  Alpha_value(ALPHA_VALUE),
                  Alpha_shape_2::REGULARIZED);  // REGULARIZED mode for "nice" shapes; GENERAL for irregular shapes. 

    std::vector<Segment> segments;
    findAlphaEdges(A, std::back_inserter(segments));
    Polygon_2 alpha_shape_polygon;
    for (const auto& segment : segments) alpha_shape_polygon.push_back(segment.source());  // For counterclockwise order of vertices.

    // TODO: Implement adding the number of points or maybe some other reference to the actual points as well (normal of plane, etc.)
}

template <class OutputIterator>
void PolygonCalculator::findAlphaEdges(const Alpha_shape_2& A, OutputIterator out){

    Alpha_shape_edges_iterator it = A.alpha_shape_edges_begin(),
                               end = A.alpha_shape_edges_end();

    if (it != end) {

        std::set<Point> visited_points; 
        Segment current_segment = A.segment(*it++);
        *out++ = current_segment;
        visited_points.insert(current_segment.source());
        visited_points.insert(current_segment.target());

        while (!visited_points.empty() && it != end) {

            bool found_next = false;
            for (Alpha_shape_edges_iterator next_it = it; next_it != end; ++next_it) {

                Segment next_segment = A.segment(*next_it);

                if (visited_points.find(next_segment.source()) != visited_points.end() &&
                    visited_points.find(next_segment.target()) != visited_points.end()) {
                    continue;
                }
                if (next_segment.source() == current_segment.target()) {

                    *out++ = next_segment;
                    visited_points.insert(next_segment.source());
                    visited_points.insert(next_segment.target());
                    current_segment = next_segment;
                    found_next = true;
                    break;
                }
            }
            if (!found_next) {
                ++it;
            }
        }
    }  
}

// Explanation in the paper by Jyh-Ming Lien, Nancy M. Amato: "Approximate convex decomposition of polygons" 
void PolygonCalculator::recursiveApproximateDecomposition(const Polygon_2& concave_hull_polygon, double depth_threshold,
                                                          std::vector<Polygon_2>& convex_polygons_to_pack){

    std::vector<Point> concave_hull_vertices;
    for (auto vertex = concave_hull_polygon.vertices_begin(); vertex != concave_hull_polygon.vertices_end(); ++vertex)
    {
        concave_hull_vertices.push_back(*vertex);
    }

    ConcaveHullPocket pocket;
    bool has_found_deep_pocket = false;

    while (!has_found_deep_pocket) {

        if (concave_hull_vertices.empty()) return;

        if (isHullConvex(concave_hull_vertices)){
            Polygon_2 convex_polygon;
            for (const auto& vertex : concave_hull_vertices) convex_polygon.push_back(vertex);
            convex_polygons_to_pack.push_back(convex_polygon);
            return;
        }

        pocket = findFirstConcaveHullPocket(concave_hull_vertices);

        if (pocket.start_bridge_index == -1) return;

        if (pocket.max_depth < depth_threshold){
            // Remove vertices exclusively between start_bridge_index and end_bridge_index
            removeAllExclusiveVertices(pocket.start_bridge_index, pocket.end_bridge_index, concave_hull_vertices);
        }
        else has_found_deep_pocket = true;
    }

    Point other_vertex_for_cutting;
    int other_vertex_index_for_cutting = findClosestIntersectionWithRay(pocket.deepest_vertex, pocket.cut_direction, pocket.end_bridge_index, 
                                                                    pocket.start_bridge_index, concave_hull_vertices, other_vertex_for_cutting);

    if (other_vertex_index_for_cutting == -1) return;  // Something went wrong with finding the other vertex for cutting

    concave_hull_vertices.insert(concave_hull_vertices.begin() + other_vertex_index_for_cutting, other_vertex_for_cutting);
    if (other_vertex_index_for_cutting < pocket.deepest_vertex_index)
        pocket.deepest_vertex_index++;

    int p1_start_index = pocket.deepest_vertex_index;
    int p1_end_index = other_vertex_index_for_cutting;
    int p2_start_index = other_vertex_index_for_cutting;
    int p2_end_index = pocket.deepest_vertex_index;

    std::vector<Point> p1(concave_hull_vertices.begin() + p1_start_index, concave_hull_vertices.begin() + p1_end_index + 1);
    std::vector<Point> p2(concave_hull_vertices.begin() + p2_start_index, concave_hull_vertices.begin() + p2_end_index + 1);

    if (p1.size() == concave_hull_vertices.size() || p2.size() == concave_hull_vertices.size())
    {
        return;  // Something went wrong with splitting the polygon
    }
    recursiveApproximateDecomposition(Polygon_2(p1.begin(), p1.end()), depth_threshold, convex_polygons_to_pack);
    recursiveApproximateDecomposition(Polygon_2(p2.begin(), p2.end()), depth_threshold, convex_polygons_to_pack);
}

// Helper function with vertices directly instead of with the resulting polygon, so that we don't have to create polygons recursively
bool isHullConvex(const std::vector<Point>& concave_hull_vertices) {

    auto get_previous_index = [&concave_hull_vertices](size_t index) -> size_t {
        return (index == 0) ? concave_hull_vertices.size() - 1 : index - 1;
    };
    auto get_next_index = [&concave_hull_vertices](size_t index) -> size_t {
        return (index + 1) % concave_hull_vertices.size();
    };

    if (concave_hull_vertices.size() <= 3) return true;

    for (size_t i = 0; i < concave_hull_vertices.size(); i++) {

        const Point& vertex = concave_hull_vertices[i];
        const Point& previous_vertex = concave_hull_vertices[get_previous_index(i)];
        const Point& next_vertex = concave_hull_vertices[get_next_index(i)];
        double cross_product = (vertex.x() - previous_vertex.x()) * (next_vertex.y() - previous_vertex.y()) -
                               (vertex.y() - previous_vertex.y()) * (next_vertex.x() - previous_vertex.x());

        // If the cross product is non-negative we have a convex angle
        if (cross_product >= 0.0) return false;
    }
    return true;
}

ConcaveHullPocket PolygonCalculator::findFirstConcaveHullPocket(const std::vector<Point>& concave_hull_vertices){
    ConcaveHullPocket pocket;
    if (concave_hull_vertices.size() < 3) {
        return pocket; 
    }
    for (size_t i = 0; i < concave_hull_vertices.size(); ++i) {
        const Point& vertex = concave_hull_vertices[i];
        const Point& prevVertex = concave_hull_vertices[(i == 0) ? concave_hull_vertices.size() - 1 : i - 1];
        const Point& nextVertex = concave_hull_vertices[(i + 1) % concave_hull_vertices.size()];

        double angle = angleBetween(prevVertex, vertex, nextVertex);

        // Check if the angle is concave (greater than 180 degrees)
        if (angle > M_PI) {
            pocket.start_bridge_index = i;
            pocket.end_bridge_index = (i + 1) % concave_hull_vertices.size(); // Next vertex index
            pocket.deepest_vertex = vertex;
            pocket.deepest_vertex_index = i;

            Point cutDirection = calculateCutDirection(prevVertex, vertex, nextVertex);
            pocket.cut_direction = cutDirection;

            double maxDepth = calculateMaxDepth(concave_hull_vertices, i, cutDirection);
            pocket.max_depth = maxDepth;

            return pocket;
        }
    }

    return pocket; // Returns an empty pocket if no concave pocket is found
}

double PolygonCalculator::angleBetween(const Point& p1, const Point& p2, const Point& p3) {
    double dx1 = p2.x() - p1.x();
    double dy1 = p2.y() - p1.y();
    double dx2 = p3.x() - p2.x();
    double dy2 = p3.y() - p2.y();

    double dotProduct = dx1 * dx2 + dy1 * dy2;
    double magnitude1 = sqrt(dx1 * dx1 + dy1 * dy1);
    double magnitude2 = sqrt(dx2 * dx2 + dy2 * dy2);

    double angle = acos(dotProduct / (magnitude1 * magnitude2));
    return angle;
}

Point PolygonCalculator::calculateCutDirection(const Point& p1, const Point& p2, const Point& p3) {
    double dx1 = p2.x() - p1.x();
    double dy1 = p2.y() - p1.y();
    double dx2 = p3.x() - p2.x();
    double dy2 = p3.y() - p2.y();

    double avgX = (dx1 + dx2) / 2;
    double avgY = (dy1 + dy2) / 2;
    double magnitude = sqrt(avgX * avgX + avgY * avgY);
    avgX /= magnitude;
    avgY /= magnitude;

    return Point(avgX, avgY);
}

double PolygonCalculator::calculateMaxDepth(const std::vector<Point>& concave_hull_vertices, int index, const Point& cut_direction) {
    double maxDepth = 0.0;

    int prevIndex = (index == 0) ? concave_hull_vertices.size() - 1 : index - 1;
    int nextIndex = (index + 1) % concave_hull_vertices.size();

    const Point& prev_vertex = concave_hull_vertices[prevIndex];
    const Point& current_vertex = concave_hull_vertices[index];
    const Point& next_vertex = concave_hull_vertices[nextIndex];

    double distance_to_edge = distanceToSegment(current_vertex, prev_vertex, next_vertex);

    // Calculate the maximum depth using the distance to the edge and the cut direction
    maxDepth = distance_to_edge / (2 * fabs(cut_direction.x() * (next_vertex.y() - prev_vertex.y()) - cut_direction.y() * (next_vertex.x() - prev_vertex.x())));

    return maxDepth;
}

double PolygonCalculator::distanceToSegment(const Point& p, const Point& p1, const Point& p2) {
    double dx = p2.x() - p1.x();
    double dy = p2.y() - p1.y();
    double dot = (p.x() - p1.x()) * dx + (p.y() - p1.y()) * dy;
    double len_sq = dx * dx + dy * dy;
    double param = dot / len_sq;

    double xx, yy;

    if (param < 0 || (p1.x() == p2.x() && p1.y() == p2.y())) {
        xx = p1.x();
        yy = p1.y();
    } else if (param > 1) {
        xx = p2.x();
        yy = p2.y();
    } else {
        xx = p1.x() + param * dx;
        yy = p1.y() + param * dy;
    }

    double dx2 = p.x() - xx;
    double dy2 = p.y() - yy;
    return sqrt(dx2 * dx2 + dy2 * dy2);
}


void PolygonCalculator::removeAllExclusiveVertices(int start_bridge_index, int end_bridge_index, std::vector<Point>& concave_hull_vertices)
{
    // Ensure indices are valid and not out of bounds
    if (start_bridge_index < 0 || start_bridge_index >= concave_hull_vertices.size() ||
        end_bridge_index < 0 || end_bridge_index >= concave_hull_vertices.size()) {
        return;
    }
    int num_vertices_to_remove = (end_bridge_index - start_bridge_index + concave_hull_vertices.size()) % concave_hull_vertices.size();
    if (num_vertices_to_remove <= 0) return;

    auto erase_start = concave_hull_vertices.begin() + (start_bridge_index + 1) % concave_hull_vertices.size();
    auto erase_end = concave_hull_vertices.begin() + end_bridge_index;
    if (erase_start < erase_end) {
        concave_hull_vertices.erase(erase_start, erase_end);
    } else {
        concave_hull_vertices.erase(erase_start, concave_hull_vertices.end());
        concave_hull_vertices.erase(concave_hull_vertices.begin(), erase_end);
    }
}

int PolygonCalculator::findClosestIntersectionWithRay(const Point& deepest_vertex, const Point& cut_direction,
                                                      int end_bridge_index, int start_bridge_index,
                                                      const std::vector<Point>& concave_hull_vertices,
                                                      Point& other_vertex_for_cutting)
{
    int closest_intersection_index = -1;
    double closest_distance = std::numeric_limits<double>::max();

    for (int i = 0; i < concave_hull_vertices.size(); ++i) {
        const Point& vertex1 = concave_hull_vertices[i];
        const Point& vertex2 = concave_hull_vertices[(i + 1) % concave_hull_vertices.size()];
        Point intersection_point = calculateIntersectionWithRay(deepest_vertex, cut_direction, vertex1, vertex2);

        // Check if the intersection point is valid
        if (!std::isnan(intersection_point.x()) && !std::isnan(intersection_point.y())) {
            double distance_to_intersection = squareDistanceBetweenPoints(deepest_vertex, intersection_point);

            if (distance_to_intersection < closest_distance) {
                closest_distance = distance_to_intersection;
                closest_intersection_index = i;
                other_vertex_for_cutting = intersection_point;
            }
        }
    }

    return closest_intersection_index;
}

Point PolygonCalculator::calculateIntersectionWithRay(const Point& ray_origin, const Point& ray_direction,
                                                      const Point& edge_start, const Point& edge_end)
{
    Point ray_vector(ray_direction.x() - ray_origin.x(), ray_direction.y() - ray_origin.y());
    Point edge_vector(edge_end.x() - edge_start.x(), edge_end.y() - edge_start.y());

    double determinant = ray_vector.x() * edge_vector.y() - ray_vector.y() * edge_vector.x();

    // If the determinant is close to zero, the ray and the edge are parallel, no intersection
    const double epsilon = 1e-20; 
    if (std::abs(determinant) < epsilon) {
        // Return a special point to indicate no intersection
        return Point(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
    }
    double t = ((edge_start.x() - ray_origin.x()) * edge_vector.y() - (edge_start.y() - ray_origin.y()) * edge_vector.x()) / determinant;
    double u = ((edge_start.x() - ray_origin.x()) * ray_vector.y() - (edge_start.y() - ray_origin.y()) * ray_vector.x()) / determinant;

    // If both t and u are within [0, 1], the intersection point is within the edge segment
    if (t >= 0 && t <= 1 && u >= 0) {
        double intersection_x = ray_origin.x() + t * ray_vector.x();
        double intersection_y = ray_origin.y() + t * ray_vector.y();
        return Point(intersection_x, intersection_y);
    }
    return Point(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
}


double PolygonCalculator::squareDistanceBetweenPoints(const Point& p1, const Point& p2)
{
    double dx = p2.x() - p1.x();
    double dy = p2.y() - p1.y();
    return std::sqrt(dx * dx + dy * dy);
}