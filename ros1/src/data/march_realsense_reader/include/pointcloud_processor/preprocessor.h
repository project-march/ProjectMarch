#ifndef MARCH_PREPROCESSOR_H
#define MARCH_PREPROCESSOR_H

#include <string>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include "yaml-cpp/yaml.h"

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;

class Preprocessor {
  public:
    Preprocessor(YAML::Node config_tree,
                 PointCloud::Ptr pointcloud,
                 Normals::Ptr pointcloud_normals);
    Preprocessor(std::string file_name,
                 PointCloud::Ptr pointcloud,
                 Normals::Ptr normal_pointcloud);
    // This function is required to be implemented by any preprocessor
    virtual void preprocess()=0;
    virtual ~Preprocessor() {};

    PointCloud::Ptr pointcloud_;
    Normals::Ptr pointcloud_normals_;
    YAML::Node config_tree_;
};

class SimplePreprocessor : Preprocessor {
  public:
    //Use the constructors defined in the super class
    using Preprocessor::Preprocessor;
    void preprocess();
};

class NormalsPreprocessor : Preprocessor {
public:
  using Preprocessor::Preprocessor; //Use the constructors defined in the super class
  void preprocess();
  void downsample();
  void removeStatisticalOutliers();
  void transformPointCloudFromUrdf();
  void transformPointCloud();
  void fillNormalCloud();
  void filterOnDistanceFromOrigin();
  void filterOnNormalOrientation();
>>>>>>> 2c7b21f6c3708e5db56c386402b8e92a95e93bed
};

#endif //MARCH_PREPROCESSOR_H