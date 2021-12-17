#ifndef MARCH_POINT_FINDER_H
#define MARCH_POINT_FINDER_H

#define RES 70

#include "utilities/math_utilities.hpp"
#include <array>
#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <string>
#include <vector>

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

class PointFinder {
public:
    explicit PointFinder(PointCloud::Ptr pointcloud, std::string left_or_right,
        Point& step_point);

    ~PointFinder() = default;

    void findPoints(std::vector<Point>* position_queue);

protected:
    PointCloud::Ptr pointcloud_;
    std::vector<double> search_dimensions_;
    std::string left_or_right_;

    int grid_resolution_ = RES;
    double cell_width = 1.0 / grid_resolution_;

    std::array<std::array<double, RES>, RES> height_map_;
    std::array<std::array<double, RES>, RES> height_map_temp_;
    std::array<std::array<double, RES>, RES> derivatives_;

    double derivative_threshold_ = 0.03;

    double optimal_foot_x_;
    double optimal_foot_y_;
    double current_foot_z_;

    double foot_width_ = 0.10;
    double foot_length_ = 0.20;
    int rect_width = ceil(foot_width_ / cell_width);
    int rect_height = ceil(foot_length_ / cell_width);

    double x_displacements_left = ceil(0.05 / cell_width);
    double x_displacements_right = ceil(0.10 / cell_width);
    double y_displacements_front = ceil(0.20 / cell_width);
    double y_displacements_far = ceil(0.05 / cell_width);

    std::vector<int> x_displacements;
    std::vector<int> y_displacements;

    double x_offset;
    double y_offset;
    double x_width;
    double y_width;

    double available_points_ratio = 0.85;

    void mapPointCloudToHeightMap();

    void interpolateMap();

    void convolveGaussianKernel();

    void convolveLaplacianKernel();

    void findFeasibleFootPlacements(std::vector<Point>* position_queue);
};

#endif // MARCH_POINT_FINDER_H