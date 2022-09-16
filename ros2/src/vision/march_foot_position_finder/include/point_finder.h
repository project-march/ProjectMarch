/**
 * @author Tuhin Das - MARCH 7
 */

#ifndef MARCH_POINT_FINDER_H
#define MARCH_POINT_FINDER_H

#define WIDTH_RES 40
#define HEIGHT_RES 80

#include "rclcpp/rclcpp.hpp"
#include "utilities/math_utilities.hpp"
#include <array>
#include <chrono>
#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <thread>
#include <vector>

using namespace marchMathUtilities;
using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

class PointFinder {
public:
    explicit PointFinder(rclcpp::Node* n, std::string left_or_right);

    ~PointFinder() = default;

    void findPoints(const PointCloud::Ptr& pointcloud, Point& step_point, std::vector<Point>* position_queue);

    void startParameterCallback(const std::vector<rclcpp::Parameter>& parameters);

    std::vector<Point> retrieveTrackPoints(const Point& start, const Point& end, int num_points);

    std::vector<double> getDisplacements();

    void readParameters(const std::vector<rclcpp::Parameter>& parameters);

    Point getOriginalPoint(int index);

    int getObstaclePenalty(int index);

protected:
    rclcpp::Node* n_;
    std::vector<double> search_dimensions_;
    std::string left_or_right_;

    int grid_height_resolution_ = HEIGHT_RES;
    int grid_width_resolution_ = WIDTH_RES;
    double cell_width_ = 1.0 / grid_height_resolution_;
    Point ORIGIN;

    std::array<std::array<double, HEIGHT_RES>, WIDTH_RES> height_map_;
    std::array<std::array<double, HEIGHT_RES>, WIDTH_RES> height_map_temp_;
    std::array<std::array<double, HEIGHT_RES>, WIDTH_RES> derivatives_;
    std::array<std::array<double, HEIGHT_RES>, WIDTH_RES> derivatives_temp_;

    double derivative_threshold_;

    double optimal_foot_x_;
    double optimal_foot_y_;
    double current_foot_z_;

    double foot_width_;
    double foot_length_;
    double actual_foot_length_;
    int rect_width_;
    int rect_height_;
    int actual_rect_height_;
    double toe_space_;
    double floating_foot_ratio_;

    double displacements_outside_;
    double displacements_inside_;
    double displacements_near_;
    double displacements_far_;

    std::vector<int> horizontal_displacements_;
    std::vector<int> vertical_displacements_;
    std::vector<int> flipping_displacements_;

    double x_offset_;
    double y_offset_;
    double x_width_;
    double y_width_;

    double max_z_distance_;
    double available_points_ratio_;
    int num_track_points_;

    std::vector<Point> original_position_queue_;
    std::vector<bool> obstacles_found_;

    bool locked_;
    bool update_arrays_;

    void initializeValues();

    void initializeSearchDimensions(Point& step_point);

    void mapPointCloudToHeightMap(const PointCloud::Ptr& pointcloud);

    void publishHeightMap();

    void interpolateMap();

    void findFeasibleFootPlacements(std::vector<Point>* position_queue);

    void computeFootPlateDisplacement(int x, int y, double height, std::vector<Point>* position_queue);

    int xCoordinateToIndex(double x);

    int yCoordinateToIndex(double y);

    double xIndexToCoordinate(int x);

    double yIndexToCoordinate(int y);
};

#endif // MARCH_POINT_FINDER_H
