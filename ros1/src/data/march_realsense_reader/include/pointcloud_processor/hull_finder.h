#ifndef MARCH_HULL_FINDER_H
#define MARCH_HULL_FINDER_H

#include "yaml-cpp/yaml.h"
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include <string>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;
using Region = pcl::PointIndices;
using PlaneCoefficients = pcl::ModelCoefficients;
using Hull = pcl::PointCloud<pcl::PointXYZ>;
using Polygon = std::vector<pcl::Vertices>;
using RegionVector = std::vector<Region>;
using PlaneCoefficientsVector = std::vector<PlaneCoefficients::Ptr>;
using HullVector = std::vector<Hull::Ptr>;
using PolygonVector = std::vector<Polygon>;

class HullFinder {
public:
    HullFinder(YAML::Node config_tree, bool debugging);
    // This function is required to be implemented by any plane finder
    virtual bool findHulls(PointCloud::Ptr pointcloud,
        Normals::Ptr normal_pointcloud,
        boost::shared_ptr<RegionVector> region_vector,
        boost::shared_ptr<PlaneCoefficientsVector> plane_coefficients_vector,
        boost::shared_ptr<HullVector> hull_vector,
        boost::shared_ptr<PolygonVector> polygon_vector)
        = 0;

    virtual ~HullFinder() {};

protected:
    PointCloud::Ptr pointcloud_;
    Normals::Ptr pointcloud_normals_;
    boost::shared_ptr<RegionVector> region_vector_;
    boost::shared_ptr<PlaneCoefficientsVector> plane_coefficients_vector_;
    boost::shared_ptr<HullVector> hull_vector_;
    boost::shared_ptr<PolygonVector> polygon_vector_;
    YAML::Node config_tree_;
    bool debugging_;
};

class CHullFinder : HullFinder {
public:
    /** Basic constructor for HullFinder preprocessor, but this will also read
     * the yaml **/
    CHullFinder(YAML::Node config_tree, bool debugging);
    /** This function should take in a pointcloud with matching normals and
     * regions, and turn this into chulls where the foot can be located. **/
    bool findHulls(PointCloud::Ptr pointcloud, Normals::Ptr normal_pointcloud,
        boost::shared_ptr<RegionVector> region_vector,
        boost::shared_ptr<PlaneCoefficientsVector> plane_coefficients_vector,
        boost::shared_ptr<HullVector> hull_vector,
        boost::shared_ptr<PolygonVector> polygon_vector) override;

protected:
    // Convert a region into a convex or concave hull
    bool getCHullFromRegion();

    // Get the points and normals of the region and initialize region variables
    bool initializeRegionVariables();

    // Get the plane coefficients of the region using average point and normal
    bool getPlaneCoefficientsRegion();

    // project the region to its plane so a convex or concave hull can be create
    bool projectRegionToPlane();

    // Create the convex or concave hull from a projected region
    bool getCHullFromProjectedRegion();

    // Add the hull to a vector together with its plane coefficients and
    // polygons
    bool addCHullToVector();

    // Calculate the average normal and point of a region
    bool getAveragePointAndNormal(std::vector<double>& average_point,
        std::vector<double>& average_normal);

    // Read all the relevant parameters from the yaml file
    void readYaml();

    bool convex;
    double alpha;
    int hull_dimension;

    int region_index_;
    PointCloud::Ptr region_points_;
    Normals::Ptr region_normals_;
    PointCloud::Ptr region_points_projected_;
    PlaneCoefficients::Ptr plane_coefficients_;
    Hull::Ptr hull_;
    Polygon polygon_;
    Region region_;
};

#endif // MARCH_HULL_FINDER_H
