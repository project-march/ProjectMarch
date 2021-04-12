#include "pointcloud_processor/parameter_determiner.h"
#include "march_shared_msgs/GaitParameters.h"
#include "utilities/output_utilities.h"
#include "utilities/realsense_gait_utilities.h"
#include "yaml-cpp/yaml.h"
#include <ctime>
#include <pcl/filters/crop_hull.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include <utilities/yaml_utilities.h>

using PointCloud2D = pcl::PointCloud<pcl::PointXY>;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointNormalCloud = pcl::PointCloud<pcl::PointNormal>;
using Normals = pcl::PointCloud<pcl::Normal>;
using Region = pcl::PointIndices;
using PlaneCoefficients = pcl::ModelCoefficients;
using Hull = pcl::PointCloud<pcl::PointXYZ>;
using Polygon = std::vector<pcl::Vertices>;
using RegionVector = std::vector<Region>;
using PlaneCoefficientsVector = std::vector<PlaneCoefficients::Ptr>;
using HullVector = std::vector<Hull::Ptr>;
using PolygonVector = std::vector<Polygon>;
using GaitParameters = march_shared_msgs::GaitParameters;

ParameterDeterminer::ParameterDeterminer(YAML::Node config_tree, bool debugging)
    : debugging_ { debugging }
    , config_tree_ { config_tree }
{
}

// Construct a basic HullParameterDeterminer class
HullParameterDeterminer::HullParameterDeterminer(
    YAML::Node config_tree, bool debugging)
    : ParameterDeterminer(config_tree, debugging)
{
    readYaml();
}

// Read all relevant parameters from the parameter yaml file
void HullParameterDeterminer::readYaml()
{
    number_of_optional_foot_locations = yaml_utilities::grabParameter<int>(
        config_tree_, "number_of_optional_foot_locations");
    general_most_desirable_location_is_mid
        = yaml_utilities::grabParameter<bool>(
            config_tree_, "general_most_desirable_location_is_mid");
    general_most_desirable_location_is_small
        = yaml_utilities::grabParameter<bool>(
            config_tree_, "general_most_desirable_location_is_small");
    if (YAML::Node stairs_locations_parameters
        = config_tree_["stairs_locations"]) {
        min_x_stairs = yaml_utilities::grabParameter<double>(
            stairs_locations_parameters, "min_x_stairs");
        max_x_stairs = yaml_utilities::grabParameter<double>(
            stairs_locations_parameters, "max_x_stairs");
        min_z_stairs = yaml_utilities::grabParameter<double>(
            stairs_locations_parameters, "min_z_stairs");
        max_z_stairs = yaml_utilities::grabParameter<double>(
            stairs_locations_parameters, "max_z_stairs");
        y_location = yaml_utilities::grabParameter<double>(
            stairs_locations_parameters, "y_location");
    } else {
        ROS_ERROR("'stairs_locations' parameters not found in parameters file");
    }
}

/** This function takes in a pointcloud with matching normals and
 * hulls, and turn this into a location where the foot can be placed,
 * from this location, gaits parameters are made. **/
bool HullParameterDeterminer::determineParameters(
    boost::shared_ptr<PlaneCoefficientsVector> const plane_coefficients_vector,
    boost::shared_ptr<HullVector> const hull_vector,
    boost::shared_ptr<PolygonVector> const polygon_vector,
    SelectedGait const selected_obstacle,
    boost::shared_ptr<GaitParameters> gait_parameters)
{
    time_t start_determine_parameters = clock();

    ROS_DEBUG("Determining parameters with simple parameter determiner");
    hull_vector_ = hull_vector;
    selected_obstacle_ = selected_obstacle;
    gait_parameters_ = gait_parameters;
    plane_coefficients_vector_ = plane_coefficients_vector;
    polygon_vector_ = polygon_vector;
    selected_obstacle_ = selected_obstacle;

    bool success = true;

    success &= getOptimalFootLocation();

    ROS_DEBUG_STREAM("The optimal foot location is "
        << output_utilities::pointToString(optimal_foot_location));

    success &= getGaitParametersFromFootLocation();

    ROS_DEBUG_STREAM("With corresponding parameters (size, height, side) ("
        << gait_parameters_->step_size_parameter << ", "
        << gait_parameters_->step_height_parameter << ", "
        << gait_parameters_->side_step_parameter << ") ");

    time_t end_determine_parameters = clock();
    double time_taken
        = double(end_determine_parameters - start_determine_parameters)
        / double(CLOCKS_PER_SEC);
    ROS_DEBUG_STREAM("Time taken by the hull parameter determiner,   is : "
        << std::fixed << time_taken << std::setprecision(5) << " sec "
        << std::endl);

    return success;
};

// Find the parameters from the foot location by finding at what percentage of
// the end points it is
bool HullParameterDeterminer::getGaitParametersFromFootLocation()
{
    if (selected_obstacle_ == SelectedGait::stairs_up) {
        gait_parameters_->step_size_parameter
            = (optimal_foot_location.x - min_x_stairs)
            / (max_x_stairs - min_x_stairs);
        gait_parameters_->step_height_parameter
            = (optimal_foot_location.z - min_z_stairs)
            / (max_z_stairs - min_z_stairs);
    } else {
        ROS_ERROR_STREAM("No way to transform a foot location to parameters "
                         "is implemented yet for obstacle "
            << selected_obstacle_);
        return false;
    }
    return true;
}

// Get the optimal foot location by finding which possible foot location is
// closest to the most desirable foot location
bool HullParameterDeterminer::getOptimalFootLocation()
{
    bool success = true;

    // Get some locations on the ground we might want to place our foot
    foot_locations_to_try = boost::make_shared<PointCloud2D>();
    success &= getOptionalFootLocations(foot_locations_to_try);

    // Crop those locations to only be left with locations where it is possible
    // to place the foot
    possible_foot_locations = boost::make_shared<PointNormalCloud>();
    success &= cropCloudToHullVector(
        foot_locations_to_try, possible_foot_locations);

    // Get the location where we would ideally place the foot
    success &= getGeneralMostDesirableLocation();

    success &= getPossibleMostDesirableLocation(possible_foot_locations);

    return success;
}

// From the possible foot locations, find which one is closes to the most
// desirable location
bool HullParameterDeterminer::getPossibleMostDesirableLocation(
    PointNormalCloud::Ptr possible_foot_locations)
{
    if (possible_foot_locations->points.size() == 0) {
        ROS_ERROR_STREAM(
            "The possible foot locations cloud is empty. "
            "Unable to compute corresponding possible foot locations");
        return false;
    }

    double min_distance_to_most_desirable_location
        = std::numeric_limits<double>::max();

    for (pcl::PointNormal& possible_foot_location : *possible_foot_locations) {
        if (not isValidLocation(possible_foot_location)) {
            continue;
        }

        double distance_to_most_desirable_location
            = (possible_foot_location.x - most_desirable_foot_location_.x)
                * (possible_foot_location.x - most_desirable_foot_location_.x)
            + (possible_foot_location.y - most_desirable_foot_location_.y)
                * (possible_foot_location.y - most_desirable_foot_location_.y)
            + (possible_foot_location.z - most_desirable_foot_location_.z)
                * (possible_foot_location.z - most_desirable_foot_location_.z);

        if (distance_to_most_desirable_location
            < min_distance_to_most_desirable_location) {
            min_distance_to_most_desirable_location
                = distance_to_most_desirable_location;
            optimal_foot_location = possible_foot_location;
        }
    }
    if (min_distance_to_most_desirable_location
        != std::numeric_limits<double>::max()) {
        return true;
    } else {
        ROS_ERROR_STREAM("No valid foot location could be found for the "
                         "current selected obstacle "
            << selected_obstacle_);
        return false;
    }
}

// Verify that the found location is valid for the requested gait
bool HullParameterDeterminer::isValidLocation(
    pcl::PointNormal possible_foot_location)
{
    if (selected_obstacle_ == SelectedGait::stairs_up) {
        // Less and larger than signs are swapped for the x coordinate
        // as the positive x axis points in the backwards direction of the
        // exoskeleton
        if (possible_foot_location.x < min_x_stairs
            && optimal_foot_location.x > max_x_stairs
            && possible_foot_location.z > min_z_stairs
            && optimal_foot_location.z < max_z_stairs) {
            return true;
        }
    } else {
        ROS_ERROR_STREAM("optimalLocationIsValid method has not been "
                         "implemented for obstacle "
            << selected_obstacle_ << ". Returning false.");
        return false;
    }
    // If no check concludes that the location is valid, return that the
    // location is invalid.
    return false;
}

// Compute the optimal foot location as if one were not limited by anything.
bool HullParameterDeterminer::getGeneralMostDesirableLocation()
{
    if (general_most_desirable_location_is_mid) {
        most_desirable_foot_location_.x = (min_x_stairs + max_x_stairs) / 2.0;
        most_desirable_foot_location_.y = y_location;
        most_desirable_foot_location_.z = (min_z_stairs + max_z_stairs) / 2.0;
    } else if (general_most_desirable_location_is_small) {
        most_desirable_foot_location_.x = min_x_stairs;
        most_desirable_foot_location_.y = y_location;
        most_desirable_foot_location_.z = min_z_stairs;
    } else {
        ROS_ERROR_STREAM(
            "No method for finding the general most desirable foot location "
            "was given. "
            "Unable to compute general most desirable foot location.");
        return false;
    }
    return true;
}

// Create a point cloud with points on the ground where the points represent
// where it should be checked if there is a valid foot location
bool HullParameterDeterminer::getOptionalFootLocations(
    PointCloud2D::Ptr foot_locations_to_try)
{
    foot_locations_to_try->points.resize(number_of_optional_foot_locations);

    if (selected_obstacle_ == SelectedGait::stairs_up) {
        for (int i = 0; i < number_of_optional_foot_locations; i++) {
            double x_location = min_x_stairs
                + (max_x_stairs - min_x_stairs) * i
                    / (double)(number_of_optional_foot_locations - 1);
            foot_locations_to_try->points[i].x = x_location;
            foot_locations_to_try->points[i].y = y_location;
        }
    } else {
        ROS_ERROR_STREAM("The selected obstacle "
            << selected_obstacle_
            << " does not have a way to create the optional foot locations to "
               "try cloud");
        return false;
    }
    return true;
}

/** For each hull, the input cloud's z coordinate is set so that it
 * lies on the corresponding plane, then the input cloud is cropped, the points
 * inside the hull (the cropped cloud) are moved to the output cloud with the
 * normal of the plane This process is repeated for each hull. If each point in
 * the input_cloud has been moved to the output cloud,
 * result is set to true, it is set to false otherwise **/
bool HullParameterDeterminer::cropCloudToHullVector(
    PointCloud2D::Ptr const input_cloud, PointNormalCloud::Ptr output_cloud)
{
    if (input_cloud->points.size() == 0) {
        ROS_WARN_STREAM("cropCloudToHullVector method called with an input "
                        "cloud of size zero. "
                        "No cropping can be done, returning.");
        return false;
    } else if (hull_vector_->size() == 0) {
        ROS_WARN_STREAM(
            "cropCloudToHull method called with emtpy hull_vector_. "
            "No cropping can be done, returning.");
        return false;
    }
    bool success = true;
    for (int hull_index = 0; hull_index < hull_vector_->size(); hull_index++) {
        PointCloud::Ptr elevated_cloud = boost::make_shared<PointCloud>();
        success &= addZCoordinateToCloudFromPlaneCoefficients(input_cloud,
            plane_coefficients_vector_->at(hull_index), elevated_cloud);

        success &= cropCloudToHull(elevated_cloud, hull_vector_->at(hull_index),
            polygon_vector_->at(hull_index));

        PointNormalCloud::Ptr elevated_cloud_with_normals
            = boost::make_shared<PointNormalCloud>();
        success &= addNormalToCloudFromPlaneCoefficients(elevated_cloud,
            plane_coefficients_vector_->at(hull_index),
            elevated_cloud_with_normals);

        *output_cloud += *elevated_cloud_with_normals;
    }

    return success;
}

// Elevate the 2D points so they have z coordinate as if they lie on the plane
// of the hull
bool HullParameterDeterminer::addZCoordinateToCloudFromPlaneCoefficients(
    PointCloud2D::Ptr const input_cloud,
    PlaneCoefficients::Ptr const plane_coefficients,
    PointCloud::Ptr elevated_cloud)
{
    elevated_cloud->points.resize(input_cloud->points.size());

    int point_index = 0;
    for (pcl::PointXYZ& elevated_point : *elevated_cloud) {
        // using z = - (d + by + ax) / c from plane equation ax + by + cz + d =
        // 0
        pcl::PointXY input_point = input_cloud->points[point_index];
        elevated_point.x = input_point.x;
        elevated_point.y = input_point.y;
        elevated_point.z = -(plane_coefficients->values[3]
                               + plane_coefficients->values[1]
                                   * input_cloud->points[point_index].y
                               + plane_coefficients->values[0]
                                   * input_cloud->points[point_index].x)
            / plane_coefficients->values[2];

        point_index++;
    }
    return true;
}

// Remove all points from a cloud which do not fall in the hull
bool HullParameterDeterminer::cropCloudToHull(
    PointCloud::Ptr elevated_cloud, const Hull::Ptr hull, const Polygon polygon)
{
    if (elevated_cloud->points.size() == 0) {
        ROS_WARN_STREAM("The cloud to be cropped in the "
                        "HullParameterDeterminer contains no points.");
        return false;
    }
    pcl::CropHull<pcl::PointXYZ> crop_filter;
    crop_filter.setInputCloud(elevated_cloud);
    crop_filter.setHullCloud(hull);
    crop_filter.setHullIndices(polygon);
    crop_filter.setDim(
        2); //////////////////////////////////////////////////////////
    crop_filter.filter(*elevated_cloud);
    return true;
}

// Add normals to the elevated cloud which correspond to the normal vector of
// the plane
bool HullParameterDeterminer::addNormalToCloudFromPlaneCoefficients(
    PointCloud::Ptr const elevated_cloud,
    PlaneCoefficients::Ptr const plane_coefficients,
    PointNormalCloud::Ptr elevated_cloud_with_normals)
{
    elevated_cloud_with_normals->width = elevated_cloud->width;
    elevated_cloud_with_normals->height = elevated_cloud->height;
    elevated_cloud_with_normals->points.resize(elevated_cloud->points.size());

    double normalising_constant
        = plane_coefficients->values[0] * plane_coefficients->values[0]
        + plane_coefficients->values[1] * plane_coefficients->values[1]
        + plane_coefficients->values[2] * plane_coefficients->values[2];

    if (normalising_constant < std::numeric_limits<double>::epsilon()) {
        ROS_ERROR_STREAM("The normal vector of the current plane is too close "
                         "to the zero vector.");
        return false;
    }

    int point_index = 0;
    for (pcl::PointNormal& elevated_point_with_normal :
        *elevated_cloud_with_normals) {
        pcl::PointXYZ elevated_point = elevated_cloud->points[point_index];
        elevated_point_with_normal.x = elevated_point.x;
        elevated_point_with_normal.y = elevated_point.y;
        elevated_point_with_normal.z = elevated_point.z;

        // using that [a b c]^T is perpendicular to the plane in plane equation
        // ax + by + cz + d = 0
        elevated_point_with_normal.normal_x
            = plane_coefficients->values[0] / normalising_constant;
        elevated_point_with_normal.normal_y
            = plane_coefficients->values[1] / normalising_constant;
        elevated_point_with_normal.normal_z
            = plane_coefficients->values[2] / normalising_constant;
        point_index++;
    }
    return true;
}

bool SimpleParameterDeterminer::determineParameters(
    boost::shared_ptr<PlaneCoefficientsVector> const plane_coefficients_vector,
    boost::shared_ptr<HullVector> const hull_vector,
    boost::shared_ptr<PolygonVector> const polygon_vector,
    SelectedGait const selected_obstacle,
    boost::shared_ptr<GaitParameters> gait_parameters)
{
    ROS_DEBUG("Determining parameters with simple parameter determiner");
    hull_vector_ = hull_vector;
    selected_obstacle_ = selected_obstacle;
    gait_parameters_ = gait_parameters;
    plane_coefficients_vector_ = plane_coefficients_vector;
    polygon_vector_ = polygon_vector;

    // Return a standard step parameter, which works for medium stairs and
    // medium ramp
    gait_parameters_->step_height_parameter = 0.5;
    gait_parameters_->step_size_parameter = 0.5;
    return true;
};