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
                 Normals::Ptr normal_pointcloud);
    Preprocessor(std::string file_name,
                 PointCloud::Ptr pointcloud,
                 Normals::Ptr normal_pointcloud);
    virtual void preprocess()=0; // This function is required to be implemented by
    // any preprocessor
    virtual ~Preprocessor() {};

    PointCloud::Ptr pointcloud_;
    Normals::Ptr normal_pointcloud_;
    YAML::Node config_tree_;
};

class SimplePreprocessor : Preprocessor {
  public:
    using Preprocessor::Preprocessor; //Use the constructors defined in the super class
    void preprocess();
};

class NormalPreprocessor : Preprocessor {
public:
  using Preprocessor::Preprocessor; //Use the constructors defined in the super class
  void preprocess();
};

#endif //MARCH_PREPROCESSOR_H