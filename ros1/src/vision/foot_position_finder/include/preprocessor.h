#ifndef MARCH_PREPROCESSOR_H
#define MARCH_PREPROCESSOR_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using NormalCloud = pcl::PointCloud<pcl::Normal>;


class Preprocessor {
public:
    explicit Preprocessor(PointCloud::Ptr pointcloud, NormalCloud::Ptr normalcloud);

    bool preprocess();


protected:

    bool voxelDownSample(double voxel_size);

    bool transformPointsToOrigin();

    bool estimateNormals(int number_of_neighbours);

    bool filterOnDistance(int x_min, int x_max, int y_min, int y_max, int z_min, int z_max);

    bool transformPointCloudFromUrdf();

    PointCloud::Ptr pointcloud_;
    NormalCloud::Ptr normalcloud_;

    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tfListener;
    
    std::string pointcloud_frame_id;

};

#endif // MARCH_PREPROCESSOR_H