#ifndef MARCH_REGION_CREATOR_H
#define MARCH_REGION_CREATOR_H

#include <string>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include "yaml-cpp/yaml.h"

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;
using RegionVector = std::vector<pcl::PointIndices>;

class RegionCreator
{
  public:
    RegionCreator(YAML::Node config_tree, bool debugging);
    // This function is required to be implemented by any region creator
    virtual bool create_regions(PointCloud::Ptr pointcloud,
                                Normals::Ptr normal_pointcloud,
                                boost::shared_ptr<RegionVector> region_vector)=0;
    virtual ~RegionCreator() {};

  protected:
    PointCloud::Ptr pointcloud_;
    Normals::Ptr normal_pointcloud_;
    boost::shared_ptr<RegionVector> region_vector_;
    YAML::Node config_tree_;
    bool debugging_;
};

class SimpleRegionCreator : RegionCreator
{
  public:
    //Use the constructors defined in the super class
    using RegionCreator::RegionCreator;
    /** This function should take in a pointcloud with matching normals and cluster them
    in regions, based on the parameters in the YAML node given at construction. **/
    bool create_regions(PointCloud::Ptr pointcloud,
                        Normals::Ptr normal_pointcloud,
                        boost::shared_ptr<RegionVector> region_vector) override;
};

#endif //MARCH_PREPROCESSOR_H