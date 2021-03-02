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
                    Normals::Ptr normal_pointcloud) override;

  protected:
    /** Calls the tf listener, to know transform at current time and transforms the
     pointcloud **/
    void transformPointCloudFromUrdf();

    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tfListener;
};

#endif //MARCH_PREPROCESSOR_H