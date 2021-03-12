#include "pointcloud_processor/region_creator.h"
#include "yaml-cpp/yaml.h"
#include <ros/ros.h>
#include <utilities/yaml_utilities.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>



using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;
using RegionVector = std::vector<pcl::PointIndices>;

// Construct a basic RegionCreator class
RegionCreator::RegionCreator(YAML::Node config_tree, bool debugging):
    config_tree_{config_tree},
    debugging_{debugging}
{

}

bool regionGrower::create_regions(PointCloud::Ptr pointcloud,
                                         Normals::Ptr pointcloud_normals,
                                         boost::shared_ptr<RegionVector>
                                         region_vector)
{
  pointcloud_ = pointcloud;
  pointcloud_normals_ = pointcloud_normals;
  region_vector_ = region_vector;
  ROS_DEBUG_STREAM("Creating regions with region growing");

  success = true;
  success &= read_yaml();
  setup_region_grower();
  success &= extract_regions();
  return success;
}
bool regionGrower::read_yaml()
{
  if (YAML::Node region_growing_parameters = config_tree_["region_growing"])
  {
    number_of_neighbours = yaml_utilities::grabParameter<double>(region_growing_parameters, "number_of_neighbours");
    min_cluster_size = yaml_utilities::grabParameter<double>(region_growing_parameters, "min_cluster_size");
    max_cluster_size = yaml_utilities::grabParameter<double>(region_growing_parameters, "max_cluster_size");
    smoothness_threshold = yaml_utilities::grabParameter<double>(region_growing_parameters, "smoothness_threshold");
    curvature_threshold = yaml_utilities::grabParameter<double>(region_growing_parameters, "curvature_threshold");
    return true;
  }
  else
  {
    ROS_ERROR("'region_growing' parameters not found in parameter file");
    return false;
  }
}

void regionGrower::setup_region_grower()
{
  pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  region_grower.setMinClusterSize(min_cluster_size);
  region_grower.setMaxClusterSize(max_cluster_size);
  region_grower.setSearchMethod(tree);
  region_grower.setNumberOfNeighbours(number_of_neighbours);
  region_grower.setInputCloud(pointcloud_);
  region_grower.setInputNormals(pointcloud_normals_);
  region_grower.setSmoothnessThreshold(smoothness_threshold);
  region_grower.setCurvatureThreshold(curvature_threshold);
}

bool regionGrower::extract_regions()
{
  try
  {
    region_grower.extract(*region_vector_);
    ROS_DEBUG("Total number of clusters found: %lu", region_vector_->size());
    if (debugging_)
    {
      int i = 0;
      for (auto region: *region_vector_)
      {
        ROS_DEBUG("Total number of points in cluster %i: %lu", i, region.indices.size());
        i++;
      }
    }
    return true;
  }
  catch(...)
  {
    ROS_ERROR("Something went wrong during extracting the regions from the region grower.");
    return false;
  }

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr regionGrower::debug_visualisation()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloured_cloud = region_grower.getColoredCloud();
  return coloured_cloud;
}