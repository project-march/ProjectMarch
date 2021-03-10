#ifndef MARCH_REGION_CREATOR_H
#define MARCH_REGION_CREATOR_H

#include <string>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include "yaml-cpp/yaml.h"
#include "pointcloud_processor/region_creator.h"
#include <pcl/segmentation/region_growing.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;
using RegionVector = std::vector<pcl::PointIndices>;

class RegionCreator
{
  public:
    RegionCreator(YAML::Node config_tree, bool debugging);
    // This function is required to be implemented by any region creator
    virtual bool create_regions(PointCloud::Ptr pointcloud,
                                Normals::Ptr pointcloud_normals,
                                boost::shared_ptr<RegionVector> regions_vector)=0;
    virtual ~RegionCreator() {};

  protected:
    PointCloud::Ptr pointcloud_;
    Normals::Ptr pointcloud_normals_;
    boost::shared_ptr<RegionVector> regions_vector_;
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
                        Normals::Ptr pointcloud_normals,
                        boost::shared_ptr<RegionVector> regions_vector) override;
};

class regionGrower : RegionCreator
{
  public:
    //Use the constructors defined in the super class
    using RegionCreator::RegionCreator;
    /** This function should take in a pointcloud with matching normals and cluster them
    in regions, based on the parameters in the YAML node given at construction. **/
    bool create_regions(PointCloud::Ptr pointcloud,
                        Normals::Ptr pointcloud_normals,
                        boost::shared_ptr<RegionVector> regions_vector) override;
    void debug_visualisation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloured_cloud);

  private:
    void setup_region_grower();
    bool read_yaml();
    bool extract_regions();

  private:
    // Region Growing Object
    pcl::RegionGrowing <pcl::PointXYZ, pcl::Normal> region_grower;

//    // Debug coloured cloud
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloured_cloud;

    // Region Growing configuration parameters
    int number_of_neighbours;
    int min_cluster_size;
    int max_cluster_size;
    double smoothness_threshold;
    double curvature_threshold;
};

#endif //MARCH_PREPROCESSOR_H