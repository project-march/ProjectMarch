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

    // Removes a point from a pointcloud (and optionally the corresponding pointcloud_normals as well) at a given index
    void removePointByIndex(int index, PointCloud::Ptr pointcloud, Normals::Ptr pointcloud_normals = nullptr);

    PointCloud::Ptr pointcloud_;
    Normals::Ptr pointcloud_normals_;
    YAML::Node config_tree_;
};

class SimplePreprocessor : Preprocessor {
  public:
    // Use the constructors defined in the super class
    using Preprocessor::Preprocessor;
    void preprocess();

  private:
  // Transforms the pointcloud so that the location and orientation of the origin match that of the foot.
  void transformPointCloudFromUrdf();
};

class NormalsPreprocessor : Preprocessor {
public:
  // Use the constructors defined in the super class
  using Preprocessor::Preprocessor;

  // Calls all subsequent methods to preprocess a pointlcoud using normal vectors
  void preprocess();

  // Removes points from the pointcloud such that there is only one point left in a certain area
  // (specified in the parameter file)
  void downsample();

  // Rotate and translates the pointcloud by some certain amounts (specified in the parameter file)
  void transformPointCloud();

  // Estimates the normals of the pointcloud and fills the pointcloud_normals_ cloud with those
  void fillNormalCloud();

  // Removes all points which are futher away then a certain distance from the origin (specified in the parameter file)
  void filterOnDistanceFromOrigin();

  // Removes all points which do not roughly have a normal in a certain direction (specified in the parameter file)
  void filterOnNormalOrientation();
};

#endif //MARCH_PREPROCESSOR_H