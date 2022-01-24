#ifndef MARCH_PREPROCESSOR_H
#define MARCH_PREPROCESSOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using NormalCloud = pcl::PointCloud<pcl::Normal>;

class Preprocessor {
public:
    explicit Preprocessor(ros::NodeHandle* n, PointCloud::Ptr pointcloud,
        NormalCloud::Ptr normalcloud);

    void preprocess();

protected:
    void voxelDownSample(float voxel_size);

    void estimateNormals(int number_of_neighbours);

    void filterOnDistance(float x_min, float x_max, float y_min, float y_max,
        float z_min, float z_max);

    void transformPointCloudFromUrdf();

    PointCloud::Ptr pointcloud_;
    NormalCloud::Ptr normalcloud_;

    float voxel_size_;
    float x_min_;
    float x_max_;
    float y_min_;
    float y_max_;
    float z_min_;
    float z_max_;

    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
    std::unique_ptr<tf2_ros::TransformListener> tfListener_;

    std::string base_frame_;
    std::string pointcloud_frame_id_;
};

#endif // MARCH_PREPROCESSOR_H
