/**
 * @author Tuhin Das - MARCH 7
 */

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <point_finder.h>

#include <vector>

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;

/**
 * Constructs a PointFinder object to find a possible foot location in a single
 * depth frame. The parameterised variables are also initialised here.
 *
 * Heights/derivatives are initialized with the values ±10, so that no
 * nonexisting points can be found by default.
 *
 * @param pointcloud a pointer to a PCL pointcloud
 * @param left_or_right whether a position should be found for the left or right
 * foot
 * @param step_point an initial desired step point
 */
// Suppress lint error "variables are not initialized" (ros parameters)
// NOLINTNEXTLINE
PointFinder::PointFinder(rclcpp::Node* n, PointCloud::Ptr pointcloud,
    std::string left_or_right, Point& step_point)
    : pointcloud_ { std::move(pointcloud) }
    , left_or_right_ { std::move(left_or_right) }
    , n_ { n }
{
    std::fill_n(&height_map_[0][0], grid_resolution_ * grid_resolution_, -10);
    std::fill_n(
        &height_map_temp_[0][0], grid_resolution_ * grid_resolution_, -10);
    std::fill_n(&derivatives_[0][0], grid_resolution_ * grid_resolution_, 10);
    std::fill_n(
        &derivatives_temp_[0][0], grid_resolution_ * grid_resolution_, 10);

    foot_width_ = n_->get_parameter("foot_width").as_double();
    foot_length_ = n_->get_parameter("foot_length").as_double();
    actual_foot_length_ = n_->get_parameter("actual_foot_length").as_double();

    displacements_outside_
        = n_->get_parameter("displacements_outside").as_double();
    displacements_inside_
        = n_->get_parameter("displacements_inside").as_double();
    displacements_near_ = n_->get_parameter("displacements_near").as_double();
    displacements_far_ = n_->get_parameter("displacements_far").as_double();

    // Convert displacements from meters to number of grid cells
    displacements_outside_ = ceil(displacements_outside_ / cell_width_);
    displacements_inside_ = ceil(displacements_inside_ / cell_width_);
    displacements_near_ = ceil(displacements_near_ / cell_width_);
    displacements_far_ = ceil(displacements_far_ / cell_width_);

    available_points_ratio_
        = n_->get_parameter("available_points_ratio").as_double();
    derivative_threshold_
        = n_->get_parameter("derivative_threshold").as_double();
    max_z_distance_ = n_->get_parameter("max_z_distance").as_double();
    num_track_points_ = n_->get_parameter("num_track_points").as_int();

    rect_width_ = ceil(foot_width_ / cell_width_);
    rect_height_ = ceil(foot_length_ / cell_width_);
    actual_rect_height_ = ceil(actual_foot_length_ / cell_width_);

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

    flipping_displacements_.push_back(0);
    for (int i = 1; i < actual_rect_height_ / 2.0; i++) {
        flipping_displacements_.push_back(i);
        flipping_displacements_.push_back(-i);
    }

    x_offset_ = -search_dimensions_[0];
    y_offset_ = -search_dimensions_[2];
    x_width_ = search_dimensions_[1] - search_dimensions_[0];
    y_width_ = search_dimensions_[3] - search_dimensions_[2];

    for (int y = 1; y <= displacements_near_; y++) {
        vertical_displacements_.push_back(y);
    }
    for (int y = 0; y >= -displacements_far_; y--) {
        vertical_displacements_.push_back(y);
    }

    if (left_or_right_ == "left") {
        for (int x = 0; x >= -displacements_inside_; x--) {
            horizontal_displacements_.push_back(x);
        }
        for (int x = 1; x <= displacements_outside_; x++) {
            horizontal_displacements_.push_back(x);
        }
    } else if (left_or_right_ == "right") {
        for (int x = 0; x <= displacements_inside_; x++) {
            horizontal_displacements_.push_back(x);
        }
        for (int x = -1; x >= -displacements_outside_; x--) {
            horizontal_displacements_.push_back(x);
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
    convolveLaplacianKernel(height_map_, derivatives_);
    findFeasibleFootPlacements(position_queue);
}

/**
 * Maps a pointcloud to a 2D matrix where only heights are inserted.
 * Indices in the matrix are found based on the x and y coordinates in 3D space.
 */
void PointFinder::mapPointCloudToHeightMap()
{
    for (std::size_t i = 0; i < pointcloud_->size(); i++) {
        Point p = pointcloud_->points[i];

        int x_index = xCoordinateToIndex(p.x);
        int y_index = yCoordinateToIndex(p.y);
        if (x_index < 0 || y_index < 0) {
            continue;
        }

        auto current_height = height_map_[y_index][x_index];
        height_map_[y_index][x_index] = std::max(current_height, (double)p.z);
    }
}

/**
 * Looks for feasible foot positions around the desired position. A preference
 * is given to points closer to the exo, or points towards the outer sides.
 */
void PointFinder::findFeasibleFootPlacements(std::vector<Point>* position_queue)
{
    for (auto& y_shift : horizontal_displacements_) {
        for (auto& x_shift : vertical_displacements_) {
            int num_free_cells = 0;
            int x_opt = xCoordinateToIndex(optimal_foot_x_) + x_shift;
            int y_opt = yCoordinateToIndex(optimal_foot_y_) + y_shift;

            for (int x = x_opt - rect_height_ / 2;
                 x < x_opt + rect_height_ / 2.0; x++) {
                for (int y = y_opt - rect_width_ / 2;
                     y < y_opt + rect_width_ / 2.0; y++) {
                    if (std::abs(derivatives_[y][x]) < derivative_threshold_) {
                        num_free_cells++;
                    }
                }
            }

            if (num_free_cells
                >= rect_height_ * rect_width_ * available_points_ratio_) {

                double height = height_map_[y_opt][x_opt];
                if (std::abs(height - current_foot_z_) <= 0.30) {
                    computeFootPlateDisplacement(
                        x_opt, y_opt, height, position_queue);
                }
            }

            if (position_queue->size() > 0) {
                return;
            }
        }
    }
}

void PointFinder::computeFootPlateDisplacement(
    int x, int y, double height, std::vector<Point>* position_queue)
{
    // Make the minimum height map value equal to the found point z-value
    for (int row = 0; row < RES; row++) {
        for (int col = 0; col < RES; col++) {
            height_map_temp_[row][col]
                = std::max(height, height_map_[row][col]);
        }
    }

    convolveLaplacianKernel(height_map_temp_, derivatives_temp_);

    for (auto& shift : flipping_displacements_) {

        int x_index = x + shift;
        int y_index = y;
        int num_free_cells = 0;

        for (int x = x_index - actual_rect_height_ / 2;
             x < x_index + actual_rect_height_ / 2.0; x++) {
            for (int y = y_index - rect_width_ / 2;
                 y < y_index + rect_width_ / 2.0; y++) {
                if (std::abs(derivatives_temp_[y][x]) < derivative_threshold_) {
                    num_free_cells++;
                }
            }
        }

        if (num_free_cells >= actual_rect_height_ * rect_width_ * 0.97) {

            double x_final = xIndexToCoordinate(x_index);
            double y_final = yIndexToCoordinate(y_index);

            if (!std::isnan(x_final) && !std::isnan(y_final)) {
                Point p = Point((float)x_final, (float)y_final, (float)height);
                position_queue->push_back(p);
                return;
            }
        }
    }
}

std::vector<Point> PointFinder::retrieveTrackPoints(
    const Point& start, const Point& end)
{
    std::vector<Point> points = linearDomain(start, end, num_track_points_);
    std::vector<Point> track_points;

    for (Point& p : points) {
        int x_index = xCoordinateToIndex(p.x);
        int y_index = yCoordinateToIndex(p.y);

        float z;

        if (x_index < 0 || y_index < 0) {
            z = 0;
        } else {
            z = height_map_[y_index][x_index];
        }

        track_points.emplace_back(Point(/*_x=*/p.x, /*_y=*/p.y, /*_z=*/z));
    }

    return track_points;
}

/**
 * Convert an x-coordinate to an x-index in the height map.
 *
 * @param x a 3D x-coordinate
 * @return x-index in height map
 */
int PointFinder::xCoordinateToIndex(double x)
{
    int index = (int)((x + x_offset_) / x_width_ * grid_resolution_);
    if (index >= RES || index < 0) {
        index = -1;
    }
    return index;
}

/**
 * Convert a y-coordinate to an y-index in the height map.
 *
 * @param y a 3D y-coordinate
 * @return y-index in height map
 */
int PointFinder::yCoordinateToIndex(double y)
{
    int index = grid_resolution_
        - (int)((y + y_offset_) / y_width_ * grid_resolution_);
    if (index >= RES || index < 0) {
        index = -1;
    }
    return index;
}

/**
 * Convert an x-index in the height map to a 3D x-coordinate.
 *
 * @param x an x-index in the height map
 * @param x 3D x-coordinate
 */
double PointFinder::xIndexToCoordinate(int x)
{
    return ((double)x / grid_resolution_) - x_offset_ + cell_width_ / 2.0;
}

/**
 * Convert an y-index in the height map to a 3D y-coordinate.
 *
 * @param y an y-index in the height map
 * @return 3D y-coordinate
 */
double PointFinder::yIndexToCoordinate(int y)
{
    return ((double)(grid_resolution_ - y) / grid_resolution_) - y_offset_
        - cell_width_ / 2.0;
}

/**
 * Retrieve the displacements used to search for a foot-shaped rectangle.
 *
 * @return the rectangle displacements
 */
std::vector<double> PointFinder::getDisplacements()
{
    double x_outside, x_inside, y_near, y_far;

    x_outside = n_->get_parameter("displacements_outside").as_double();
    x_inside = n_->get_parameter("displacements_inside").as_double();
    y_near = n_->get_parameter("displacements_near").as_double();
    y_far = n_->get_parameter("displacements_far").as_double();

    return std::vector<double> { x_outside, x_inside, y_near, y_far };
}
