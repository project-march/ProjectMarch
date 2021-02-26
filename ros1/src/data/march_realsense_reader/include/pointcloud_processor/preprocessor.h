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

    void removePointByIndex(int index, PointCloud::Ptr pointcloud, Normals::Ptr pointcloud_normals = nullptr);

    PointCloud::Ptr pointcloud_;
    Normals::Ptr pointcloud_normals_;
    YAML::Node config_tree_;
};

class SimplePreprocessor : Preprocessor {
  public:
    //Use the constructors defined in the super class
    using Preprocessor::Preprocessor;
    void preprocess();

  private:
    void transformPointCloudFromUrdf();
};

class NormalsPreprocessor : Preprocessor {
public:
  using Preprocessor::Preprocessor; //Use the constructors defined in the super class
  void preprocess();
  void downsample();
  void transformPointCloud();
  void fillNormalCloud();
  void filterOnDistanceFromOrigin();
  void filterOnNormalOrientation();
};

#endif //MARCH_PREPROCESSOR_H