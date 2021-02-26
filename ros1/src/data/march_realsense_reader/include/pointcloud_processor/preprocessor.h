#ifndef MARCH_PREPROCESSOR_H
#define MARCH_PREPROCESSOR_H

#include <string>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include "yaml-cpp/yaml.h"
#include <tf2_ros/transform_listener.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;

class Preprocessor {
  public:
    Preprocessor(YAML::Node config_tree);

    // This function is required to be implemented by any preprocessor
    virtual void preprocess(PointCloud::Ptr pointcloud,
                            Normals::Ptr normal_pointcloud)=0;
    virtual ~Preprocessor() {};

    PointCloud::Ptr pointcloud_;
    Normals::Ptr normal_pointcloud_;
    YAML::Node config_tree_;
};

class SimplePreprocessor : Preprocessor {
  public:
    SimplePreprocessor(YAML::Node config_tree);
    void preprocess(PointCloud::Ptr pointcloud,
                    Normals::Ptr normal_pointcloud);

  protected:
    void transformPointCloudFromUrdf();
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tfListener;
};

#endif //MARCH_PREPROCESSOR_H