#include <iostream>
#include <foot_position_finder.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <iomanip>

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;

FootPositionFinder::FootPositionFinder(PointCloud::Ptr pointcloud,
                                       std::vector<double>& search_dimensions,
                                       char left_or_right)
    : pointcloud_ { pointcloud },
      search_dimensions_ { search_dimensions },
      left_or_right { left_or_right }
    {
        memset(height_map_temp_, 0, sizeof(int) * grid_resolution_ * grid_resolution_);
        memset(height_map_, 0, sizeof(double) * grid_resolution_ * grid_resolution_);
        memset(derivatives_, 1, sizeof(double) * grid_resolution_ * grid_resolution_);
        if (rect_width % 2 == 0) rect_width--;
        if (rect_height % 2 == 0) rect_height--;
        if (left_or_right == 'l')
        {
            optimal_foot_x_ *= -1;
            auto temp = x_displacements_left;
            x_displacements_left = -x_displacements_right;
            x_displacements_left = -x_displacements_left;
        }
    }

bool FootPositionFinder::findFootPositions(std::vector<Point> *position_queue)
{
    mapPointCloudToHeightMap();
    convolveLaplacianKernel();
    findFeasibleFootPlacements(position_queue);
    return true;
}

bool FootPositionFinder::mapPointCloudToHeightMap()
{
    double x_offset = -search_dimensions_[0];
    double y_offset = -search_dimensions_[2];
    double x_width = search_dimensions_[1] - search_dimensions_[0];
    double y_width = search_dimensions_[3] - search_dimensions_[2];

    for(std::size_t i = 0; i < pointcloud_->size(); i++)
    {
        auto p = pointcloud_->points[i];

        int x_index = (int) ((p.x + x_offset) / x_width * grid_resolution_);
        int y_index = (int) ((p.y + y_offset) / y_width * grid_resolution_);

        auto current_height = height_map_[grid_resolution_ - y_index][x_index];

        if (current_height < 0.000001 && current_height > -0.000001)
            height_map_[grid_resolution_ - y_index][x_index] = p.z;
        else
            height_map_[grid_resolution_ - y_index][x_index] = std::max(current_height, (double) p.z);
    }

    return true;
}

bool FootPositionFinder::interpolateMap()
{
    for(int i = 0; i < grid_resolution_; i++)
        for(int j = 0; j < grid_resolution_; j++)
            if (height_map_[i][j] == 0.0)
                height_map_[i][j] = (height_map_[i + 1][j]
                                   + height_map_[i - 1][j]
                                   + height_map_[i][j + 1]
                                   + height_map_[i][j - 1]) / 4.0;
    return true;
}

bool FootPositionFinder::convolveLaplacianKernel()
{
    double laplacian[3][3] =
        {{-1, -1, -1},
         {-1, 8, -1},
         {-1, -1, -1}};

    convolve2D(laplacian, height_map_, derivatives_);
    return true;
}

template<int R>
bool FootPositionFinder::convolve2D(double kernel[3][3], double (&source)[R][R], double (&destination)[R][R])
{
    for (int i = 1; i < grid_resolution_ - 1; i++)
        for (int j = 1; j < grid_resolution_ - 1; j++)
        {
            double sum = 0;
            for (int a = 0; a < 3; a++)
                for (int b = 0; b < 3; b++)
                    sum += kernel[a][b] * source[i+(a-1)][j+(b-1)];
            destination[i][j] = sum;
        }
    return true;
}

bool FootPositionFinder::findFeasibleFootPlacements(std::vector<Point> *position_queue)
{
    auto x_offset = -search_dimensions_[0];
    auto y_offset = -search_dimensions_[2];
    double x_width = search_dimensions_[1] - search_dimensions_[0];
    double y_width = search_dimensions_[3] - search_dimensions_[2];

    std::vector<Point> points;

    std::vector<int> x_displacements;
    std::vector<int> y_displacements;

    for (int y = 0; y >= -y_displacements_front; y--)
        y_displacements.push_back(y);
    for (int y = 1; y <= y_displacements_far; y++)
        y_displacements.push_back(y);
    for (int x = 0; x <= x_displacements_right; x++)
        x_displacements.push_back(x);
    for (int x = -1; x >= -x_displacements_left; x--)
        x_displacements.push_back(x);
    
    for (auto &x_shift : x_displacements)
    {
        for (auto &y_shift : y_displacements)
        {
            int num_free_cells = 0;
            int x_opt = (int) ((optimal_foot_x_ + x_offset) / x_width * grid_resolution_) + x_shift;
            int y_opt = (int) ((optimal_foot_y_ + y_offset) / y_width * grid_resolution_) - y_shift;

            for (int x = x_opt - rect_width/2; x < x_opt + rect_width/2 + 1; x++)
            {
                for (int y = y_opt - rect_height/2; y < y_opt + rect_height/2 + 1; y++)
                {
                    if (std::abs(derivatives_[y][x]) < derivative_threshold_) {
                        num_free_cells++;
                    }
                }
            }

            if (num_free_cells >= rect_height * rect_width * available_points_ratio)
            {
                double x = ((double) x_opt / grid_resolution_) - x_offset + cell_width/2;
                double y = ((double) (grid_resolution_ - y_opt) / grid_resolution_) - y_offset - cell_width/2;
                double z = height_map_[y_opt][x_opt];
                position_queue->push_back(Point(x, y, z));
            }
        }
    }
    return true;
}

double (*FootPositionFinder::getDerivatives())[RES]
{
    return derivatives_;
}

double (*FootPositionFinder::getHeights())[RES]
{
    return height_map_;
}

