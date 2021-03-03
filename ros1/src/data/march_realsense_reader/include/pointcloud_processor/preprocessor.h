#ifndef MARCH_PREPROCESSOR_H
#define MARCH_PREPROCESSOR_H

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
                            Normals::Ptr pointcloud_normals)=0;
    virtual ~Preprocessor() {};

    // Removes a point from a pointcloud (and optionally the corresponding pointcloud_normals as well) at a given index
    void removePointByIndex(int const index, PointCloud::Ptr pointcloud, Normals::Ptr pointcloud_normals = nullptr);

    PointCloud::Ptr pointcloud_;
    Normals::Ptr pointcloud_normals_;
    YAML::Node config_tree_;
};

/** The SimplePreprocessor is mostly for debug purposes, it does only the most vital
 * step of transforming the pointcloud based on the /tf topic. More complex
 * preoprocessors will be made for normal usecases **/
class SimplePreprocessor : Preprocessor {
  public:
  /** Basic constructor for simple preprocessor, this will also create a tf_listener
  that is required for transforming the pointcloud **/
  SimplePreprocessor(YAML::Node config_tree);

  // Preprocess the given pointcloud, based on parameters in the config tree
  void preprocess(PointCloud::Ptr pointcloud,
                  Normals::Ptr pointcloud_normals) override;

protected:
  /** Calls the tf listener, to know transform at current time and transforms the
   pointcloud **/
  void transformPointCloudFromUrdf();

  std::unique_ptr<tf2_ros::Buffer> tfBuffer;
  std::unique_ptr<tf2_ros::TransformListener> tfListener;
};

class NormalsPreprocessor : Preprocessor {
public:
  /** Basic constructor for simple preprocessor, this will also create a tf_listener
  that is required for transforming the pointcloud **/
  NormalsPreprocessor(YAML::Node config_tree);

  // Calls all subsequent methods to preprocess a pointlcoud using normal vectors
  void preprocess(PointCloud::Ptr pointcloud,
                  Normals::Ptr pointcloud_normals) override;

protected:
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

  // Remove statistical outliers from the pointcloud to reduce noise
  void removeStatisticalOutliers();
};

#endif //MARCH_PREPROCESSOR_H