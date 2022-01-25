/**
 * @author Tuhin Das - MARCH 7
 */

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <point_finder.h>

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;

/**
 * Constructs a PointFinder object to find a possible foot location in a single
 * depth frame. The parameterised variables are also initialised here.
 *
 * @param pointcloud a pointer to a PCL pointcloud
 * @param left_or_right whether a position should be found for the left or right
 * foot
 * @param step_point an initial desired step point
 */
// NOLINTNEXTLINE
PointFinder::PointFinder(ros::NodeHandle* n, PointCloud::Ptr pointcloud,
    std::string left_or_right, Point& step_point)
    : pointcloud_ { std::move(pointcloud) }
    , left_or_right_ { std::move(left_or_right) }
{
    std::fill_n(&height_map_[0][0], grid_resolution_ * grid_resolution_, -5);
    std::fill_n(
        &height_map_temp_[0][0], grid_resolution_ * grid_resolution_, -5);
    std::fill_n(&derivatives_[0][0], grid_resolution_ * grid_resolution_, 1);

    ros::param::get("~foot_width", foot_width_);
    ros::param::get("~foot_length", foot_length_);

    ros::param::get("~x_displacements_left", x_displacements_left_);
    ros::param::get("~x_displacements_right", x_displacements_right_);
    ros::param::get("~y_displacements_front", y_displacements_front_);
    ros::param::get("~x_displacements_left", y_displacements_far_);

    x_displacements_left_ = ceil(x_displacements_left_ / cell_width_);
    x_displacements_right_ = ceil(x_displacements_right_ / cell_width_);
    y_displacements_front_ = ceil(y_displacements_front_ / cell_width_);
    y_displacements_far_ = ceil(y_displacements_far_ / cell_width_);

    ros::param::get("~available_points_ratio", available_points_ratio_);
    ros::param::get("~derivative_threshold", derivative_threshold_);
    ros::param::get("~max_z_distance", max_z_distance_);

    rect_width_ = ceil(foot_width_ / cell_width_);
    rect_height_ = ceil(foot_length_ / cell_width_);

    optimal_foot_x_ = step_point.x;
    optimal_foot_y_ = step_point.y;
    current_foot_z_ = step_point.z;

    search_dimensions_ = { optimal_foot_x_ - 0.5, optimal_foot_x_ + 0.5,
        optimal_foot_y_ - 0.5, optimal_foot_y_ + 0.5, -1, 1 };

    if (rect_width_ % 2 == 0) {
        rect_width_--;
    }
    if (rect_height_ % 2 == 0) {
        rect_height_--;
    }
    if (left_or_right_ == "left") {
        auto temp = x_displacements_left_;
        x_displacements_left_ = x_displacements_right_;
        x_displacements_right_ = temp;
    }

    x_offset_ = -search_dimensions_[0];
    y_offset_ = -search_dimensions_[2];
    x_width_ = search_dimensions_[1] - search_dimensions_[0];
    y_width_ = search_dimensions_[3] - search_dimensions_[2];

    for (int y = 0; y >= -y_displacements_front_; y--) {
        y_displacements_.push_back(y);
    }
    for (int y = 1; y <= y_displacements_far_; y++) {
        y_displacements_.push_back(y);
    }
    if (left_or_right_ == "left") {
        for (int x = 0; x >= -x_displacements_left_; x--) {
            x_displacements_.push_back(x);
        }
        for (int x = 1; x <= x_displacements_right_; x++) {
            x_displacements_.push_back(x);
        }
    } else if (left_or_right_ == "right") {
        for (int x = 0; x <= x_displacements_right_; x++) {
            x_displacements_.push_back(x);
        }
        for (int x = -1; x >= -x_displacements_left_; x--) {
            x_displacements_.push_back(x);
        }
    }
}

/**
 * Finds possible stepping points in the pointcloud and inserts them in a queue
 *
 * @param position_queue a pointer to a queue with possible foot positions
 */
void PointFinder::findPoints(std::vector<Point>* position_queue)
{
    mapPointCloudToHeightMap();
    convolveGaussianKernel();
    convolveLaplacianKernel();
    findFeasibleFootPlacements(position_queue);
}

/**
 * Maps a pointcloud to a 2D matrix where only heights are inserted.
 * Indices in the matrix are found based on the x and y coordinates in 3D space.
 */
void PointFinder::mapPointCloudToHeightMap()
{
    for (std::size_t i = 0; i < pointcloud_->size(); i++) {
        auto p = pointcloud_->points[i];

        int x_index = (int)((p.x + x_offset_) / x_width_ * grid_resolution_);
        int y_index = (int)((p.y + y_offset_) / y_width_ * grid_resolution_);

        auto current_height
            = height_map_temp_[grid_resolution_ - y_index][x_index];
        height_map_temp_[grid_resolution_ - y_index][x_index]
            = std::max(current_height, (double)p.z);
    }
}

/**
 * Smooths the height matrix by convolving a Gaussian kernel.
 */
void PointFinder::convolveGaussianKernel()
{
    std::array<std::array<double, 3>, 3> gaussian = {
        { { 1.0 / 16, 2.0 / 16, 1.0 / 16 }, { 2.0 / 16, 4.0 / 16, 2.0 / 16 },
            { 1.0 / 16, 2.0 / 16, 1.0 / 16 } }
    };

    convolve2D(gaussian, height_map_temp_, height_map_);
}

/**
 * Computed the second derivatives of the height matrix with a Laplacian kernel.
 */
void PointFinder::convolveLaplacianKernel()
{

    std::array<std::array<double, 3>, 3> laplacian
        = { { { 1 / 6.0, 4 / 6.0, 1 / 6.0 }, { 4 / 6.0, -20 / 6.0, 4 / 6.0 },
            { 1 / 6.0, 4 / 6.0, 1 / 6.0 } } };

    convolve2D(laplacian, height_map_, derivatives_);
}

/**
 * Looks for feasible foot positions around the desired position. A preference
 * is given to points closer to the exo, or points towards the outer sides.
 */
void PointFinder::findFeasibleFootPlacements(std::vector<Point>* position_queue)
{
    for (auto& x_shift : x_displacements_) {
        for (auto& y_shift : y_displacements_) {
            int num_free_cells = 0;
            int x_opt = (int)((optimal_foot_x_ + x_offset_) / x_width_
                            * grid_resolution_)
                + x_shift;
            int y_opt = (int)((optimal_foot_y_ + y_offset_) / y_width_
                            * grid_resolution_)
                - y_shift;

            for (int x = x_opt - rect_width_ / 2; x < x_opt + rect_width_ / 2.0;
                 x++) {
                for (int y = y_opt - rect_height_ / 2;
                     y < y_opt + rect_height_ / 2.0; y++) {
                    if (std::abs(derivatives_[y][x]) < derivative_threshold_) {
                        num_free_cells++;
                    }
                }
            }

            // Suppress lint errors: narrowing conversions (they are safe)
            if (num_free_cells
                >= rect_height_ * rect_width_ * available_points_ratio_) {
                float x = ((float)x_opt / (float)grid_resolution_)
                    - (float)x_offset_ + (float)cell_width_ / 2.0;
                float y = (((float)grid_resolution_ - (float)y_opt)
                              / (float)grid_resolution_)
                    - (float)y_offset_ - (float)cell_width_ / 2.0;
                float z = height_map_[y_opt][x_opt];

                if (std::abs(z - current_foot_z_) <= max_z_distance_
                    && !std::isnan(x) && !std::isnan(y) && !std::isnan(z)) {
                    position_queue->push_back(Point(x, y, z));
                }
            }
        }
    }
}
