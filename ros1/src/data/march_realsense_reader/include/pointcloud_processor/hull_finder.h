#ifndef MARCH_HULL_FINDER_H
#define MARCH_HULL_FINDER_H

#include <string>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include "yaml-cpp/yaml.h"

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;
using RegionVector = std::vector<pcl::PointIndices>;
using PlaneParameterVector = std::vector<pcl::ModelCoefficients::Ptr>;
using HullVector = std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>;
using PolygonVector = std::vector<std::vector<pcl::Vertices>>;

class HullFinder
{
public:
    HullFinder(YAML::Node config_tree, bool debugging);
    // This function is required to be implemented by any plane finder
    virtual bool find_hulls(PointCloud::Ptr pointcloud,
                             Normals::Ptr normal_pointcloud,
                             boost::shared_ptr<RegionVector> region_vector,
                             boost::shared_ptr<PlaneParameterVector>
                                 plane_parameter_vector,
                             boost::shared_ptr<HullVector> hull_vector,
                             boost::shared_ptr<PolygonVector> polygon_vector)=0;

    virtual ~HullFinder() {};

protected:
    PointCloud::Ptr pointcloud_;
    Normals::Ptr normal_pointcloud_;
    boost::shared_ptr<RegionVector> region_vector_;
    boost::shared_ptr<PlaneParameterVector> plane_parameter_vector_;
    boost::shared_ptr<HullVector> hull_vector_;
    boost::shared_ptr<PolygonVector> polygon_vector_;
    YAML::Node config_tree_;
    bool debugging_;
};

class SimpleHullFinder : HullFinder
{
public:
    //Use the constructors defined in the super class
    using HullFinder::HullFinder;
    /** This function should take in a pointcloud with matching normals and
     * regions, and turn this into chulls where the foot can be located. **/
    bool find_hulls(PointCloud::Ptr pointcloud,
                     Normals::Ptr normal_pointcloud,
                     boost::shared_ptr<RegionVector> region_vector,
                     boost::shared_ptr<PlaneParameterVector> plane_parameter_vector,
                     boost::shared_ptr<HullVector> hull_vector,
                     boost::shared_ptr<PolygonVector> polygon_vector) override;
};

#endif //MARCH_HULL_FINDER_H
