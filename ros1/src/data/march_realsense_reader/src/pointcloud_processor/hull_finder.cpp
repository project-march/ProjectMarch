#include "pointcloud_processor/hull_finder.h"
#include "utilities/linear_algebra_utilities.h"
#include "utilities/output_utilities.h"
#include <cmath>
#include <ros/ros.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

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

// Construct a basic HullFinder class
HullFinder::HullFinder(bool debugging)
    : debugging_ { debugging }
{
}

// Construct a basic CHullFinder class
CHullFinder::CHullFinder(bool debugging)
    : HullFinder(debugging)
    , output_plane_information(false)
{
}

bool CHullFinder::findHulls(PointCloud::Ptr pointcloud, Normals::Ptr pointcloud_normals,
    boost::shared_ptr<PointsVector> points_vector, boost::shared_ptr<NormalsVector> normals_vector,
    boost::shared_ptr<PlaneCoefficientsVector> plane_coefficients_vector, boost::shared_ptr<HullVector> hull_vector,
    boost::shared_ptr<PolygonVector> polygon_vector)
{
    time_t start_find_hulls = clock();

    pointcloud_ = pointcloud;
    pointcloud_normals_ = pointcloud_normals;
    points_vector_ = points_vector;
    normals_vector_ = normals_vector;
    plane_coefficients_vector_ = plane_coefficients_vector;
    hull_vector_ = hull_vector;
    polygon_vector_ = polygon_vector;

    ROS_DEBUG("Finding hulls with CHullFinder, C for convex or concave");

    for (region_index_ = 0; region_index_ < points_vector_->size(); region_index_++) {
        bool success = getCHullFromRegion();

        // Add the hull to a vector together with its plane coefficients and
        // polygons if it is valid
        if (hull_->points.size() == 0) {
            ROS_WARN_STREAM("Hull of region " << region_index_
                                              << " Consists of zero points, not adding it the the hull "
                                                 "vector.");
        } else if (!success) {
            ROS_WARN_STREAM(
                "Computation of hull of region " << region_index_ << " went wrong, not adding it the the hull vector.");
        } else {
            addCHullToVector();
        }
    }

    ROS_DEBUG_STREAM("The number of hulls found is: " << hull_vector_->size());
    if (hull_vector_->size() != plane_coefficients_vector_->size() || hull_vector_->size() != polygon_vector_->size()) {
        ROS_ERROR_STREAM("The hull vector does not have the same size as either "
                         "the plane coefficients vector or"
                         "the polygon vector. Returning with false.");
        return false;
    } else if (hull_vector_->size() == 0) {
        ROS_ERROR_STREAM("No hulls were found. Returning with false.");
        return false;
    }

    time_t end_find_hulls = clock();
    double time_taken = double(end_find_hulls - start_find_hulls) / double(CLOCKS_PER_SEC);
    ROS_DEBUG_STREAM("Time taken by the pointcloud HullFinder CHullFinder  is : "
        << std::fixed << time_taken << std::setprecision(5) << " sec " << std::endl);

    return true;
}
void CHullFinder::readParameters(march_realsense_reader::pointcloud_parametersConfig& config)
{
    convex = config.hull_finder_convex;
    alpha = config.hull_finder_alpha;
    hull_dimension = config.hull_dimension;
    output_plane_information = config.hull_finder_output_plane_information;

    debugging_ = config.debug;
}

// Converts a region into a convex or concave hull
bool CHullFinder::getCHullFromRegion()
{
    bool success = true;

    // Get the points and normals of the region and initialize region variables
    success &= initializeRegionVariables();

    // Get the plane coefficients of the region
    success &= getPlaneCoefficientsRegion();

    // Project the region to its plane so a convex or concave hull can be
    // created
    success &= projectRegionToPlane();

    // Create the hull
    success &= getCHullFromProjectedRegion();

    return success;
}

// Get the points and normals of the region and initialize region variables
bool CHullFinder::initializeRegionVariables()
{
    region_points_ = points_vector_->at(region_index_);
    region_normals_ = normals_vector_->at(region_index_);
    region_points_projected_ = boost::make_shared<PointCloud>();
    plane_coefficients_ = boost::make_shared<PlaneCoefficients>();
    hull_ = boost::make_shared<Hull>();
    polygon_.clear();
    return true;
}

// Get the plane coefficients of the region using average point and normal
bool CHullFinder::getPlaneCoefficientsRegion()
{
    // calculate average normal and average point to calculate plane
    // coefficients from
    std::vector<double> average_normal(/*__n=*/3);
    std::vector<double> average_point(/*__n=*/3);

    bool success = getAveragePointAndNormal(average_point, average_normal);

    // Plane coefficients given as [0]*x + [1]*y + [2]*z + [3] = 0
    plane_coefficients_->values.resize(/*__new_size=*/4);
    // The first three coefficients are the normal vector of the plane as
    // all the vectors in the plane are perpendicular to the normal
    plane_coefficients_->values.assign(average_normal.begin(), average_normal.end());
    // the final coefficient can be calculated with the plane equation ax + by +
    // cz + d = 0
    plane_coefficients_->values.push_back(
        -linear_algebra_utilities::dotProductVector<double>(average_point, average_normal));

    if (success && debugging_ && output_plane_information) {
        ROS_DEBUG_STREAM("Region " << region_index_ << " has plane coefficients: "
                                   << output_utilities::vectorToString(plane_coefficients_->values));
    }

    return success;
}

// project the region to its plane so a convex or concave hull can be created
bool CHullFinder::projectRegionToPlane()
{
    // Create the projection object
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(region_points_);
    proj.setModelCoefficients(plane_coefficients_);
    proj.filter(*region_points_projected_);
    return true;
}

// Create the convex or concave hull from a projected region and its
// corresponding polygons
bool CHullFinder::getCHullFromProjectedRegion()
{
    if (region_points_projected_->size() < 3) {
        ROS_WARN_STREAM("A minimum of 3 points are needed to construct a hull. "
                        "The number of points projected to the region plane is "
            << region_points_projected_->size() << ". Ignoring region.");
        return false;
    }
    if (convex) {
        pcl::ConvexHull<pcl::PointXYZ> convex_hull;
        convex_hull.setInputCloud(region_points_projected_);
        convex_hull.setDimension(hull_dimension);
        convex_hull.reconstruct(*hull_, polygon_);
    } else {
        pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
        concave_hull.setAlpha(alpha);
        concave_hull.setInputCloud(region_points_projected_);
        concave_hull.setDimension(hull_dimension);
        concave_hull.reconstruct(*hull_, polygon_);
    }
    return true;
}

// Add the hull to a vector together with its plane coefficients and polygons
void CHullFinder::addCHullToVector()
{
    hull_vector_->push_back(hull_);
    polygon_vector_->push_back(polygon_);
    plane_coefficients_vector_->push_back(plane_coefficients_);
}

// Calculate the average normal and point of a region
bool CHullFinder::getAveragePointAndNormal(std::vector<double>& average_point, std::vector<double>& average_normal)
{
    bool success = true;
    // Initialize the average point and normal to zero
    std::fill(average_point.begin(), average_point.end(), 0);
    std::fill(average_normal.begin(), average_normal.end(), 0);

    int number_of_points = region_points_->points.size();
    int number_of_normals = region_normals_->points.size();

    if (number_of_points == number_of_normals) {
        for (int p = 0; p < number_of_points; p++) {
            pcl::Normal current_normal = region_normals_->points[p];
            pcl::PointXYZ current_point = region_points_->points[p];

            average_normal[0] += current_normal.normal_x;
            average_normal[1] += current_normal.normal_y;
            average_normal[2] += current_normal.normal_z;

            average_point[0] += current_point.x;
            average_point[1] += current_point.y;
            average_point[2] += current_point.z;
        }
        average_point[0] /= number_of_points;
        average_point[1] /= number_of_points;
        average_point[2] /= number_of_points;

        success &= linear_algebra_utilities::normalize3DVector<double>(average_normal);
    } else {
        ROS_WARN_STREAM("Region with index " << region_index_
                                             << " does not have the same number of points and normals, "
                                                " unable to calculate average point and normal. "
                                                "Number of points: "
                                             << number_of_points << ". Number of normals: " << number_of_normals);
        return false;
    }

    return success;
}
