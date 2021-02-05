#include <pointcloud_processor/preprocessor.h>
#include "yaml-cpp/yaml.h"

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;

Preprocessor::Preprocessor(YAML::Node config_tree,
                           std::shared_ptr<PointCloud> pointcloud,
                           std::shared_ptr<Normals> normal_pointcloud):
                           config_tree_{config_tree},
                           pointcloud_{pointcloud},
                           normal_pointcloud_{normal_pointcloud}
{

}

Preprocessor::Preprocessor(
    std::string file_name,
    std::shared_ptr<PointCloud> pointcloud,
    std::shared_ptr<Normals> normal_pointcloud):
    pointcloud_{pointcloud},
    normal_pointcloud_{normal_pointcloud}
{
  std::string path = ros::package::getPath("march_realsense_reader") +
      "/config/" + file_name;
  config_tree_ = YAML::LoadFile(path);
}

void SimplePreprocessor::preprocess()
{

}
