#ifndef MARCH_REGION_CREATOR_H
#define MARCH_REGION_CREATOR_H

#include <string>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include "yaml-cpp/yaml.h"

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;
using RegionsVector = std::vector<pcl::PointIndices>;

class RegionCreator {
  public:
    RegionCreator(YAML::Node config_tree);
    // This function is required to be implemented by any region creator
    virtual void create_regions(PointCloud::Ptr pointcloud,
                                Normals::Ptr normal_pointcloud,
                                boost::shared_ptr<RegionsVector> regions_vector)=0;
    virtual ~RegionCreator() {};

  protected:
    PointCloud::Ptr pointcloud_;
    Normals::Ptr normal_pointcloud_;
    boost::shared_ptr<RegionsVector> regions_vector_;
    YAML::Node config_tree_;
};

class SimpleRegionCreator : RegionCreator {
  public:
    //Use the constructors defined in the super class
    using RegionCreator::RegionCreator;
    void create_regions(PointCloud::Ptr pointcloud,
                        Normals::Ptr normal_pointcloud,
                        boost::shared_ptr<RegionsVector> regions_vector);
};

#endif //MARCH_PREPROCESSOR_H