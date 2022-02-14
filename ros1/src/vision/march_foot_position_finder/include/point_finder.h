/**
 * @author Tuhin Das - MARCH 7
 */

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
    explicit PointFinder(ros::NodeHandle* n, PointCloud::Ptr pointcloud,
        std::string left_or_right, Point& step_point);

    ~PointFinder() = default;

    void findPoints(std::vector<Point>* position_queue);

    std::vector<double> getDisplacements();

protected:
    PointCloud::Ptr pointcloud_;
    std::vector<double> search_dimensions_;
    std::string left_or_right_;

    int grid_resolution_ = RES;
    double cell_width_ = 1.0 / grid_resolution_;

    std::array<std::array<double, RES>, RES> height_map_;
    std::array<std::array<double, RES>, RES> height_map_temp_;
    std::array<std::array<double, RES>, RES> derivatives_;

    double derivative_threshold_;

    double optimal_foot_x_;
    double optimal_foot_y_;
    double current_foot_z_;

    double foot_width_;
    double foot_length_;
    int rect_width_;
    int rect_height_;

    double x_displacements_outside_;
    double x_displacements_inside_;
    double y_displacements_near_;
    double y_displacements_far_;

    std::vector<int> x_displacements_;
    std::vector<int> y_displacements_;

    double x_offset_;
    double y_offset_;
    double x_width_;
    double y_width_;

    double max_z_distance_;
    double available_points_ratio_;

    void mapPointCloudToHeightMap();

    void interpolateMap();

    void convolveGaussianKernel();

    void convolveLaplacianKernel();

    void findFeasibleFootPlacements(std::vector<Point>* position_queue);
};

#endif // MARCH_POINT_FINDER_H
