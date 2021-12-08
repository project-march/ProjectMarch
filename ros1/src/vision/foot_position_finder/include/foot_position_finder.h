#ifndef MARCH_HEIGHT_MAP_GENERATOR_H
#define MARCH_HEIGHT_MAP_GENERATOR_H

#define RES 70

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <cmath>
#include <vector>

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

class FootPositionFinder {
public:

    explicit FootPositionFinder(PointCloud::Ptr pointcloud,
                                std::vector<double>& search_dimensions,
                                char left_or_right);

    ~FootPositionFinder() = default;

    bool findFootPositions(std::vector<Point> *position_queue);

    double (*getDerivatives())[RES];

    double (*getHeights())[RES];

protected:

    PointCloud::Ptr pointcloud_;
    std::vector<double> search_dimensions_;
    char left_or_right;

    int grid_resolution_ = RES;
    double cell_width = 1.0 / grid_resolution_;

    double height_map_[RES][RES];
    double height_map_temp_[RES][RES];
    double derivatives_[RES][RES];

    double derivative_threshold_ = 0.03;

    double optimal_foot_x_ = 0.0;
    double optimal_foot_y_ = 0.4;

    double foot_width_ = 0.10;
    double foot_length_ = 0.20;
    int rect_width = ceil(foot_width_ / cell_width);
    int rect_height = ceil(foot_length_ / cell_width);

    double x_displacements_left = ceil(0.05 / cell_width);
    double x_displacements_right = ceil(0.10 / cell_width);
    double y_displacements_front = ceil(0.20 / cell_width);
    double y_displacements_far = ceil(0.05 / cell_width);

    double x_offset;
    double y_offset;
    double x_width;
    double y_width;

    double available_points_ratio = 0.85;

    bool mapPointCloudToHeightMap();

    bool interpolateMap();

    bool convolveGaussianKernel();

    bool convolveLaplacianKernel();

    template<int K, int R>
    bool convolve2D(double kernel[K][K], double (&source)[R][R], double (&destination)[R][R]);

    bool findFeasibleFootPlacements(std::vector<Point> *position_queue);    

};

#endif // MARCH_HEIGHT_MAP_GENERATOR_H