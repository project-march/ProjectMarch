/**
 * @author Tuhin Das - MARCH 7
 */

#ifndef MARCH_PREPROCESSOR_H
#define MARCH_PREPROCESSOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using NormalCloud = pcl::PointCloud<pcl::Normal>;

class Preprocessor {
public:
    explicit Preprocessor(PointCloud::Ptr pointcloud,
        std::string& left_or_right, tf::TransformListener& listener);

    void preprocess();

protected:
    void voxelDownSample(float voxel_size);

    void estimateNormals(int number_of_neighbours);

    void filterOnDistance(float x_min, float x_max, float y_min, float y_max,
        float z_min, float z_max);

    void transformPointCloudToBaseframe();

    PointCloud::Ptr pointcloud_;

    float voxel_size_;
    float x_min_;
    float x_max_;
    float y_min_;
    float y_max_;
    float z_min_;
    float z_max_;

    std::string base_frame_;
    std::string pointcloud_frame_id_;
    tf::StampedTransform transform_;
};

#endif // MARCH_PREPROCESSOR_H
