#ifndef MARCH_REGION_CREATOR_H
#define MARCH_REGION_CREATOR_H

#include <string>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include "yaml-cpp/yaml.h"
#include <pcl/segmentation/region_growing.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;
using RegionVector = std::vector<pcl::PointIndices>;

class RegionCreator
{
  public:
    RegionCreator(YAML::Node config_tree, bool debugging);
    // This function is required to be implemented by any region creator
    virtual bool createRegions(PointCloud::Ptr pointcloud,
                                Normals::Ptr pointcloud_normals,
                                boost::shared_ptr<RegionVector> region_vector)=0;
    virtual ~RegionCreator() {};
    virtual pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_visualisation()=0;

  protected:
    PointCloud::Ptr pointcloud_;
    Normals::Ptr pointcloud_normals_;
    boost::shared_ptr<RegionVector> region_vector_;
    YAML::Node config_tree_;
    bool debugging_;
};

class RegionGrower : RegionCreator
{
  public:
    //Use the constructors defined in the super class
    RegionGrower(YAML::Node config_tree, bool debugging);
    /** Create cluster using the region growing algorithm, takes algorithm configuration from the YAML, and fills
     * parameter region_vector with clusters. **/
    bool createRegions(PointCloud::Ptr pointcloud,
                        Normals::Ptr pointcloud_normals,
                        boost::shared_ptr<RegionVector> region_vector) override;

    /**
     * @return A pointer to a single pointcloud, with unique colours for every cluster
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_visualisation() override;

  private:
    /**
    * Read out YAML
    * @return true if succesful
    */
    void readYaml();

    /**
     * Configure region growing algorithm
     */
    bool setupRegionGrower();

    /**
     * Extract clusters from region_grower object
     * @return true if succesful
     */
    bool extractRegions();

    // Region Growing Object
    pcl::RegionGrowing <pcl::PointXYZ, pcl::Normal> region_grower;

    // Region Growing configuration parameters
    int number_of_neighbours;
    int min_cluster_size;
    int max_cluster_size;
    double smoothness_threshold;
    double curvature_threshold;
};

#endif //MARCH_PREPROCESSOR_H