#ifndef MARCH_HULL_FINDER_H
#define MARCH_HULL_FINDER_H

#include <string>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include "yaml-cpp/yaml.h"

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;
using Region = pcl::PointIndices;
using PlaneCoefficients = pcl::ModelCoefficients::Ptr;
using Hull = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using Polygon = std::vector<pcl::Vertices>;
using RegionVector = std::vector<Region>;
using PlaneCoefficientsVector = std::vector<PlaneCoefficients>;
using HullVector = std::vector<Hull>;
using PolygonVector = std::vector<Polygon>;

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

class CHullFinder : HullFinder
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

protected:
    // Converts a region into a convex or concave hull
    bool getCHullFromRegion();

    // Get the plane coefficients of the region using average point and normal
    bool getPlaneCoefficientsRegion();

    // project the region to its plane so a convex or concave hull can be create
    bool projectRegionToPlane();

    // Create the convex or concave hull from a projected region
    bool getCHullFromProjectedPlane();

    // Add the hull to a vector together with its plane coefficients and polygons
    bool addCHullToVector();

    // Reads all the relevant parameters from the yaml file
    bool readYaml();

    // Plane coefficients given as [0]*x + [1]*y + [2]*z + [3] = 0
    pcl::ModelCoefficients::Ptr  plane_coefficients;

    int region_index;
    int region_vector_length;
    PointCloud::Ptr region_points_;
    PonitCloud::Ptr region_normals_;

};

#endif //MARCH_HULL_FINDER_H
