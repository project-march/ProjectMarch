#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <preprocessor.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h> 
#include <pcl/filters/extract_indices.h> 
#include <math.h>
#include <ros/console.h>


using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;
using Normal = pcl::Normal;
using NormalCloud = pcl::PointCloud<Normal>;

Preprocessor::Preprocessor(PointCloud::Ptr pointcloud, NormalCloud::Ptr normalcloud)
    : pointcloud_ { pointcloud }
    , normalcloud_ { normalcloud }
    {}

NormalsPreprocessor::NormalsPreprocessor(PointCloud::Ptr pointcloud, NormalCloud::Ptr normalcloud)
    : Preprocessor(pointcloud, normalcloud)
    {
        tfBuffer = std::make_unique<tf2_ros::Buffer>();
        tfListener = std::make_unique<tf2_ros::TransformListener>(*tfBuffer);
    }

bool NormalsPreprocessor::preprocess()
{
    voxelDownSample(0.01);
    // estimateNormals(1);
    transformPointCloudFromUrdf();
    // transformPointsToOrigin();
    filterOnDistance(-1, 1, -1, 1, -1, 1);
    return true;
}

bool NormalsPreprocessor::voxelDownSample(double voxel_size) 
{
    pcl::VoxelGrid<Point> voxel_grid;
    voxel_grid.setInputCloud(pointcloud_);
    voxel_grid.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel_grid.filter(*pointcloud_);
    return true;
}

bool NormalsPreprocessor::estimateNormals(int number_of_neighbours)
{
    pcl::NormalEstimation<Point, Normal> normal_estimator;
    normal_estimator.setInputCloud(pointcloud_);
    pcl::search::Search<Point>::Ptr tree(new pcl::search::KdTree<Point>);
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setKSearch(number_of_neighbours);
    normal_estimator.compute(*normalcloud_);
    return true;
}

bool NormalsPreprocessor::transformPointsToOrigin()
{
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    // test files
    // transform.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()));
    // transform.rotate(Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitZ()));
    // transform.rotate(Eigen::AngleAxisf(-M_PI/4*3, Eigen::Vector3f::UnitY()));

    // transform.rotate(Eigen::AngleAxisf(-M_PI/4*3, Eigen::Vector3f::UnitX()));
    // transform.rotate(Eigen::AngleAxisf(-61/180 * M_PI, Eigen::Vector3f::UnitY()));
    


    // transform.translation() << 0, -0.0725, 0;
    // transform.rotate(Eigen::AngleAxisf(0.0756695435357118, Eigen::Vector3f::UnitX()));
    // transform.translation() << 0.151, 0, 0;
    // transform.rotate(Eigen::AngleAxisf(0.000989348615771038, Eigen::Vector3f::UnitY()));
    // transform.translation() << 0, 0.03, 0.41;
    // transform.rotate(Eigen::AngleAxisf(-0.009588377793440255, Eigen::Vector3f::UnitY()));
    // transform.translation() << 0, 0.08, 0.39;
    // transform.rotate(Eigen::AngleAxisf(-0.1330429972669268, Eigen::Vector3f::UnitY()));
    pcl::transformPointCloud (*pointcloud_, *pointcloud_, transform);
    return true;
}

bool NormalsPreprocessor::filterOnDistance(int x_min, int x_max, int y_min, int y_max,  
                                           int z_min, int z_max)
{
    pcl::PointIndices::Ptr remove_indices(new pcl::PointIndices());
    pcl::ExtractIndices<Point> extract;

    for (std::size_t i = 0; i < (*pointcloud_).size(); i++)
    {
        Point pt = pointcloud_->points[i];
        if (pt.y <= y_min || pt.y >= y_max
         || pt.x <= x_min || pt.x >= x_max
         || pt.z <= z_min || pt.z >= z_max) 
        {
            remove_indices->indices.push_back(i);
        } 
    }   

    extract.setInputCloud(pointcloud_);
    extract.setIndices(remove_indices);
    extract.setNegative(true);
    extract.filter(*pointcloud_);
    return true;
}


bool NormalsPreprocessor::transformPointCloudFromUrdf()
{
    geometry_msgs::TransformStamped transform_stamped;
    try {
        pointcloud_frame_id = pointcloud_->header.frame_id.c_str();
        if (tfBuffer->canTransform("world", pointcloud_frame_id, ros::Time(), ros::Duration(1.0))) {
            transform_stamped = tfBuffer->lookupTransform("world", pointcloud_frame_id, ros::Time(0));
        }
        std::cout << transform_stamped.transform.translation << std::endl;
        std::cout << transform_stamped.transform.rotation << std::endl;
        pcl_ros::transformPointCloud(*pointcloud_, *pointcloud_, transform_stamped.transform);
    } catch (tf2::TransformException& ex) {
        ROS_WARN_STREAM("Something went wrong when transforming the pointcloud: " << ex.what());
        return false;
    }
    
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(*pointcloud_, *pointcloud_, transform);
    return true;
}
