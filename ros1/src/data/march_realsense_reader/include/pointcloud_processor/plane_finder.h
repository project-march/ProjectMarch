#ifndef MARCH_PLANE_FINDER_H
#define MARCH_PLANE_FINDER_H

#include <string>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include "yaml-cpp/yaml.h"

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;
using RegionsVector = std::vector<pcl::PointIndices>;
using PlaneParameters = std::vector<pcl::ModelCoefficients::Ptr>;
using HullsVector = std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>;

class PlaneFinder
{
public:
    PlaneFinder(YAML::Node config_tree, bool debugging);
    // This function is required to be implemented by any plane finder
    virtual bool find_planes(PointCloud::Ptr pointcloud,
                             Normals::Ptr normal_pointcloud,
                             boost::shared_ptr<RegionsVector> regions_vector,
                             boost::shared_ptr<PlaneParameters> plane_parameters,
                             boost::shared_ptr<HullsVector> hulls)=0;

    virtual ~PlaneFinder() {};

protected:
    PointCloud::Ptr pointcloud_;
    Normals::Ptr normal_pointcloud_;
    boost::shared_ptr<RegionsVector> regions_vector_;
    boost::shared_ptr<PlaneParameters> plane_parameters_;
    boost::shared_ptr<HullsVector> hulls_;
    YAML::Node config_tree_;
    bool debugging_;
};

class SimplePlaneFinder : PlaneFinder
{
public:
    //Use the constructors defined in the super class
    using PlaneFinder::PlaneFinder;
    /** This function should take in a pointcloud with matching normals and
     * regions, and turn this into chulls where the foot can be located. **/
    bool find_planes(PointCloud::Ptr pointcloud,
                     Normals::Ptr normal_pointcloud,
                     boost::shared_ptr<RegionsVector> regions_vector,
                     boost::shared_ptr<PlaneParameters> plane_parameters,
                     boost::shared_ptr<HullsVector> hulls) override;
};

#endif //MARCH_PLANE_FINDER_H
