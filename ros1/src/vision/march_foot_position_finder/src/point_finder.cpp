#include <iostream>
#include <point_finder.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <iomanip>

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;

/**
 * Constructs a PointFinder object to find a possible foot location in a single depth frame.
 * The parameterised variables are also initialised here.
 * 
 * @param pointcloud a pointer to a PCL pointcloud
 * @param left_or_right whether a position should be found for the left or right foot
 * @param step_point an initial desired step point
 */
PointFinder::PointFinder(PointCloud::Ptr pointcloud,
                         std::string left_or_right,
                         Point& step_point)
    : pointcloud_    { pointcloud },
      left_or_right_ { left_or_right }
{
    memset(height_map_, -5, sizeof(double) * grid_resolution_ * grid_resolution_);
    memset(height_map_temp_, -5, sizeof(double) * grid_resolution_ * grid_resolution_);
    memset(derivatives_, 1, sizeof(double) * grid_resolution_ * grid_resolution_);

    optimal_foot_x_ = step_point.x;
    optimal_foot_y_ = step_point.y;
    current_foot_z_ = step_point.z;

    search_dimensions_ = {optimal_foot_x_-0.5, optimal_foot_x_+0.5, optimal_foot_y_-0.5, optimal_foot_y_+0.5, -1, 1};

    if (rect_width % 2 == 0) rect_width--;
    if (rect_height % 2 == 0) rect_height--;
    if (left_or_right_ == "left")
    {
        auto temp = x_displacements_left;
        x_displacements_left = x_displacements_right;
        x_displacements_right = temp;
    }

    x_offset = -search_dimensions_[0];
    y_offset = -search_dimensions_[2];
    x_width = search_dimensions_[1] - search_dimensions_[0];
    y_width = search_dimensions_[3] - search_dimensions_[2];

    for (int y =  0; y >= -y_displacements_front; y--) y_displacements.push_back(y);
    for (int y =  1; y <=  y_displacements_far  ; y++) y_displacements.push_back(y);

    if (left_or_right_ == "left") {    
        for (int x = 0; x >= -x_displacements_left ; x--) x_displacements.push_back(x);
        for (int x = 1; x <=  x_displacements_right; x++) x_displacements.push_back(x);
    } else if (left_or_right_ == "right") {   
        for (int x =  0; x <=  x_displacements_right; x++) x_displacements.push_back(x);
        for (int x = -1; x >= -x_displacements_left ; x--) x_displacements.push_back(x);
    }
}


/**
 * Finds possible stepping points in the pointcloud and inserts them in a queue
 * 
 * @param position_queue a pointer to a queue with possible foot positions
 * @return bool whether the function succeeded
 */
bool PointFinder::findPoints(std::vector<Point> *position_queue)
{
    bool success = true;
    success &= mapPointCloudToHeightMap();
    success &= convolveGaussianKernel();
    success &= convolveLaplacianKernel();
    success &= findFeasibleFootPlacements(position_queue);
    return success;
}


/**
 * Maps a pointcloud to a 2D matrix where only heights are inserted.
 * Indices in the matrix are found based on the x and y coordinates in 3D space.
 * 
 * @return bool whether the function succeeded
 */
bool PointFinder::mapPointCloudToHeightMap()
{
    for (std::size_t i = 0; i < pointcloud_->size(); i++)
    {
        auto p = pointcloud_->points[i];
        
        int x_index = (int) ((p.x + x_offset) / x_width * grid_resolution_);
        int y_index = (int) ((p.y + y_offset) / y_width * grid_resolution_);

        auto current_height = height_map_temp_[grid_resolution_ - y_index][x_index];
        height_map_temp_[grid_resolution_ - y_index][x_index] = std::max(current_height, (double) p.z);
    }
    return true;
}


/**
 * Smooths the height matrix by convolving a Gaussian kernel.
 * 
 * @return bool whether the function succeeded
 */
bool PointFinder::convolveGaussianKernel()
{
    double gaussian[3][3] =
        {{1.0/16, 2.0/16, 1.0/16},
         {2.0/16, 4.0/16, 2.0/16},
         {1.0/16, 2.0/16, 1.0/16}};

    convolve2D(gaussian, height_map_temp_, height_map_);
    return true;
}


/**
 * Computed the second derivatives of the height matrix with a Laplacian kernel.
 * 
 * @return bool whether the function succeeded
 */
bool PointFinder::convolveLaplacianKernel()
{
    double laplacian[3][3] =
        {{1/6.0,   4/6.0, 1/6.0},
         {4/6.0, -20/6.0, 4/6.0},
         {1/6.0,   4/6.0, 1/6.0}};

    convolve2D(laplacian, height_map_, derivatives_);
    return true;
}


/**
 * Looks for feasible foot positions around the desired position. A preference is given 
 * to points closer to the exo, or points towards the outer sides. 
 * 
 * @return bool whether the function succeeded
 */
bool PointFinder::findFeasibleFootPlacements(std::vector<Point> *position_queue)
{
    for (auto &x_shift : x_displacements)
    {
        for (auto &y_shift : y_displacements)
        {
            int num_free_cells = 0;
            int x_opt = (int) ((optimal_foot_x_ + x_offset) / x_width * grid_resolution_) + x_shift;
            int y_opt = (int) ((optimal_foot_y_ + y_offset) / y_width * grid_resolution_) - y_shift;

            for (int x = x_opt - rect_width/2; x < x_opt + rect_width/2.0; x++)
                for (int y = y_opt - rect_height/2; y < y_opt + rect_height/2.0; y++)
                    if (std::abs(derivatives_[y][x]) < derivative_threshold_)
                        num_free_cells++;

            if (num_free_cells >= rect_height * rect_width * available_points_ratio)
            {
                double x = ((double) x_opt / grid_resolution_) - x_offset + cell_width/2.0;
                double y = ((double) (grid_resolution_ - y_opt) / grid_resolution_) - y_offset - cell_width/2.0;
                double z = height_map_[y_opt][x_opt];
                
                if (std::abs(z - current_foot_z_) <= 0.25 && !std::isnan(x) && !std::isnan(y) && !std::isnan(z))
                    position_queue->push_back(Point(x, y, z));
            }
        }
    }
    return true;
}
