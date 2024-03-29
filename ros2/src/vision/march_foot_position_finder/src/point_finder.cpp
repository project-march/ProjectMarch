/**
 * @author Tuhin Das - MARCH 7
 */

#include <algorithm>
#include <cmath>
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
 * depth frame.
 *
 * Heights/derivatives are initialized with the values ±10, so that no
 * nonexisting points can be found by default.
 *
 * @param n ROS node instance.
 * @param left_or_right Whether a position should be found for the left or right
 * foot.
 * @param step_point An initial desired step point.
 */
// Suppress lint error "variables are not initialized" (ros parameters)
// NOLINTNEXTLINE
PointFinder::PointFinder(rclcpp::Node* n, std::string left_or_right)
    : n_ { n }
    , left_or_right_ { std::move(left_or_right) }
{
    std::fill_n(&height_map_[0][0], grid_width_resolution_ * grid_height_resolution_, -10);
    std::fill_n(&height_map_temp_[0][0], grid_width_resolution_ * grid_height_resolution_, -10);
    std::fill_n(&derivatives_[0][0], grid_width_resolution_ * grid_height_resolution_, 10);
    std::fill_n(&derivatives_temp_[0][0], grid_width_resolution_ * grid_height_resolution_, 10);

    foot_width_ = n_->get_parameter("foot_width").as_double();
    foot_length_ = n_->get_parameter("foot_length").as_double();
    actual_foot_length_ = n_->get_parameter("actual_foot_length").as_double();

    displacements_outside_ = n_->get_parameter("displacements_outside").as_double();
    displacements_inside_ = n_->get_parameter("displacements_inside").as_double();
    displacements_near_ = n_->get_parameter("displacements_near").as_double();
    displacements_far_ = n_->get_parameter("displacements_far").as_double();

    available_points_ratio_ = n_->get_parameter("available_points_ratio").as_double();
    derivative_threshold_ = n_->get_parameter("derivative_threshold").as_double();
    max_z_distance_ = n_->get_parameter("max_z_distance").as_double();
    num_track_points_ = n_->get_parameter("num_track_points").as_int();

    ORIGIN = Point(/*_x=*/0, /*_y=*/0, /*_z=*/0);

    initializeValues();

    locked_ = false;
    update_arrays_ = false;
}

/**
 * Retrieve parameter values that are dynamically reconfigured and update the
 * values in the class.
 *
 * @param parameters Instance containing updated parameters.
 */
void PointFinder::readParameters(const std::vector<rclcpp::Parameter>& parameters)
{
    for (const auto& param : parameters) {
        if (param.get_name() == "foot_width") {
            foot_width_ = param.as_double();
        } else if (param.get_name() == "foot_length") {
            foot_length_ = param.as_double();
        } else if (param.get_name() == "actual_foot_length") {
            actual_foot_length_ = param.as_double();
        } else if (param.get_name() == "displacements_outside") {
            displacements_outside_ = param.as_double();
        } else if (param.get_name() == "displacements_inside") {
            displacements_inside_ = param.as_double();
        } else if (param.get_name() == "displacements_near") {
            displacements_near_ = param.as_double();
        } else if (param.get_name() == "displacements_far") {
            displacements_far_ = param.as_double();
        } else if (param.get_name() == "available_points_ratio") {
            available_points_ratio_ = param.as_double();
        } else if (param.get_name() == "derivative_threshold") {
            derivative_threshold_ = param.as_double();
        } else if (param.get_name() == "max_z_distance") {
            max_z_distance_ = param.as_double();
        } else if (param.get_name() == "num_track_points") {
            num_track_points_ = param.as_int();
        } else if (param.get_name() == "floating_foot_ratio") {
            floating_foot_ratio_ = param.as_double();
        } else if (param.get_name() == "toe_space") {
            toe_space_ = param.as_double();
        }
        // RCLCPP_INFO(n_->get_logger(),
        //     "\033[92mParameter %s updated in %s Point Finder\033[0m",
        //     param.get_name().c_str(), left_or_right_.c_str());
    }
    update_arrays_ = true;
}

/**
 * Initialize class attributes that determine the search region.
 */
void PointFinder::initializeValues()
{
    // Convert displacements from meters to number of grid cells
    int displacements_outside_int_ = ceil(displacements_outside_ / cell_width_);
    int displacements_inside_int_ = ceil(displacements_inside_ / cell_width_);
    int displacements_near_int_ = ceil(displacements_near_ / cell_width_);
    int displacements_far_int_ = ceil(displacements_far_ / cell_width_);

    rect_width_ = ceil(foot_width_ / cell_width_);
    rect_height_ = ceil(foot_length_ / cell_width_);
    actual_rect_height_ = ceil(actual_foot_length_ / cell_width_);

    if (rect_width_ % 2 == 0) {
        rect_width_--;
    }
    if (rect_height_ % 2 == 0) {
        rect_height_--;
    }

    flipping_displacements_.clear();
    horizontal_displacements_.clear();
    vertical_displacements_.clear();

    for (int i = 0; i <= ceil((floating_foot_ratio_ * actual_foot_length_ + toe_space_) / cell_width_); i += 2) {
        flipping_displacements_.push_back(-i);
    }

    if (left_or_right_ == "left") {
        for (int x = 0; x >= -displacements_inside_int_; x -= 2) {
            horizontal_displacements_.push_back(x);
        }
        for (int x = 1; x <= displacements_outside_int_; x += 2) {
            horizontal_displacements_.push_back(x);
        }
    } else if (left_or_right_ == "right") {
        for (int x = 0; x <= displacements_inside_int_; x += 2) {
            horizontal_displacements_.push_back(x);
        }
        for (int x = -1; x >= -displacements_outside_int_; x -= 2) {
            horizontal_displacements_.push_back(x);
        }
    }

    for (int y = 1; y <= displacements_near_int_; y += 2) {
        vertical_displacements_.push_back(y);
    }
    for (int y = 0; y >= -displacements_far_int_; y -= 2) {
        vertical_displacements_.push_back(y);
    }
}

/**
 * Determine the area around a desired point where positions will be found.
 *
 * @param step_point Desired stepping point.
 */
void PointFinder::initializeSearchDimensions(Point& step_point)
{
    optimal_foot_x_ = step_point.x;
    optimal_foot_y_ = step_point.y;
    current_foot_z_ = step_point.z;

    search_dimensions_
        = { optimal_foot_x_ - 0.5, optimal_foot_x_ + 0.5, optimal_foot_y_ - 0.25, optimal_foot_y_ + 0.25 };

    x_offset_ = -search_dimensions_[0];
    y_offset_ = -search_dimensions_[2];
    x_width_ = search_dimensions_[1] - search_dimensions_[0];
    y_width_ = search_dimensions_[3] - search_dimensions_[2];
}

/**
 * Finds possible stepping points in the pointcloud and inserts them in a queue
 *
 * @param pointcloud Pointcloud pointer where points will be found.
 * @param step_point Desired stepping point.
 * @param position_queue Queue with possible foot positions.
 */
void PointFinder::findPoints(const PointCloud::Ptr& pointcloud, Point& step_point, std::vector<Point>* position_queue)
{
    while (locked_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    locked_ = true;

    original_position_queue_.clear();
    obstacles_found_.clear();

    if (update_arrays_) {
        initializeValues();
        update_arrays_ = false;
    }

    initializeSearchDimensions(step_point);
    mapPointCloudToHeightMap(pointcloud);
    convolveLaplacianKernel(height_map_, derivatives_);
    findFeasibleFootPlacements(position_queue);
    locked_ = false;
}

/**
 * Maps a pointcloud to a 2D matrix where only heights are inserted.
 * Indices in the matrix are found based on the x and y coordinates in 3D space.
 *
 * @param pointcloud Pointcloud pointer where points will be found.
 */
void PointFinder::mapPointCloudToHeightMap(const PointCloud::Ptr& pointcloud)
{
    std::fill_n(&height_map_[0][0], grid_width_resolution_ * grid_height_resolution_, -10);

    for (std::size_t i = 0; i < pointcloud->size(); i++) {
        Point p = pointcloud->points[i];

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
 *
 * @param position_queue Queue with possible foot positions.
 */
void PointFinder::findFeasibleFootPlacements(std::vector<Point>* position_queue)
{
    for (auto& y_shift : horizontal_displacements_) {
        for (auto& x_shift : vertical_displacements_) {
            int num_free_cells = 0;
            int x_opt = xCoordinateToIndex(optimal_foot_x_) + x_shift;
            int y_opt = yCoordinateToIndex(optimal_foot_y_) + y_shift;
            int toe_space_cells = ceil(toe_space_ / cell_width_);

            for (int x = x_opt - toe_space_cells; x < x_opt + rect_height_; x++) {
                int row_cell_count = 0;
                for (int y = y_opt - rect_width_ / 2; y < y_opt + rect_width_ / 2.0; y++) {
                    if (std::abs(derivatives_[y][x]) < derivative_threshold_) {
                        row_cell_count++;
                        num_free_cells++;
                    }
                }
            }
            if (num_free_cells >= (rect_height_ + toe_space_cells) * rect_width_ * available_points_ratio_) {
                double height = height_map_[y_opt][x_opt];
                if (std::abs(height - current_foot_z_) <= 0.28) {
                    computeFootPlateDisplacement(x_opt, y_opt, height, position_queue);
                }
            }

            if (position_queue->size() > 0 && std::abs((position_queue->back()).z) < 0.05) {
                return;
            }
        }
    }
}

/**
 * Determine whether the exoskeleton steps with its toes or heels on the found
 * point (or with a point in between).
 *
 * @param position_queue Queue with possible foot positions.
 */
void PointFinder::computeFootPlateDisplacement(int x, int y, double height, std::vector<Point>* position_queue)
{

    double x_original = xIndexToCoordinate(x);
    double y_original = yIndexToCoordinate(y);
    Point original = Point((float)x_original, (float)y_original, (float)height);

    std::vector<Point> track = retrieveTrackPoints(ORIGIN, original, /*num_points=*/14);
    bool obstacle_found = false;
    for (Point& p : track) {
        if (p.z > height + 0.03) {
            obstacle_found = true;
            break;
        }
    }

    // Do not shift foot if not stepping down a stairs
    // if (height >= 0 && !obstacle_found && validatePoint(original)) {
    if (height > -0.17 && validatePoint(original)) {
        position_queue->push_back(original);
        original_position_queue_.push_back(original);
        obstacles_found_.push_back(obstacle_found);
        return;
    }

    // Make the minimum height map value equal to the found point z-value
    for (int i = 0; i < WIDTH_RES; i++) {
        for (int j = 0; j < HEIGHT_RES; j++) {
            height_map_temp_[i][j] = std::max(height, height_map_[i][j]);
        }
    }

    convolveLaplacianKernel(height_map_temp_, derivatives_temp_);

    for (auto& shift : flipping_displacements_) {

        int x_index = x + shift;
        int y_index = y;
        int num_free_cells = 0;

        for (int x = x_index; x < x_index + actual_rect_height_; x++) {
            for (int y = y_index - rect_width_ / 2; y < y_index + rect_width_ / 2.0; y++) {
                if (std::abs(derivatives_temp_[y][x]) < derivative_threshold_) {
                    num_free_cells++;
                }
            }
        }

        if (num_free_cells >= actual_rect_height_ * rect_width_ * 0.97) {

            double x_final = xIndexToCoordinate(x_index);
            double y_final = yIndexToCoordinate(y_index);

            Point p = Point((float)x_final, (float)y_final, (float)height);
            if (validatePoint(p)) {
                position_queue->push_back(p);
                original_position_queue_.push_back(original);
                obstacles_found_.push_back(obstacle_found);
                return;
            }
        }
    }
}

/**
 * Retrieve a list of points between the start and end position of the moving
 * leg.
 *
 * @param start Start point.
 * @param end End point.
 * @return std::vector<Point> A list of pointcloud points.
 */
std::vector<Point> PointFinder::retrieveTrackPoints(const Point& start, const Point& end, int num_points)
{
    if (num_points == 0) {
        num_points = num_track_points_;
    }
    std::vector<Point> points = linearDomain(start, end, num_points);
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
    int index = (int)((x + x_offset_) / x_width_ * grid_height_resolution_);
    if (index >= HEIGHT_RES || index < 0) {
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
    int index = grid_width_resolution_ - (int)((y + y_offset_) / y_width_ * grid_width_resolution_);
    if (index >= WIDTH_RES || index < 0) {
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
    return ((double)x / grid_height_resolution_) * x_width_ - x_offset_ + cell_width_ / 2.0;
}

/** * Convert an y-index in the height map to a 3D y-coordinate.
 *
 * @param y an y-index in the height map
 * @return 3D y-coordinate
 */
double PointFinder::yIndexToCoordinate(int y)
{
    return ((double)(grid_width_resolution_ - y) / grid_width_resolution_) * y_width_ - y_offset_ - cell_width_ / 2.0;
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

/**
 * Return an original point.
 */
Point PointFinder::getOriginalPoint(int index)
{
    return original_position_queue_[index];
}

/**
 * Return whether there is an obstacle in shape of a penalty.
 */
int PointFinder::getObstaclePenalty(int index)
{
    if (obstacles_found_[index]) {
        return 1;
    } else {
        return 0;
    }
}
