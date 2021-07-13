#include "utilities/realsense_category_utilities.h"
#include "yaml-cpp/yaml.h"
#include <cmath>
#include <ctime>
#include <filesystem>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pointcloud_processor/preprocessor.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_listener.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;
using namespace std::filesystem;

// Base constructor for preprocessors
Preprocessor::Preprocessor(bool debugging)
    : debugging_ { debugging }
{
}

// Create a simple preprocessor with the ability to look up transforms
SimplePreprocessor::SimplePreprocessor(bool debugging)
    : Preprocessor(debugging)
{
    tfBuffer = std::make_unique<tf2_ros::Buffer>();
    tfListener = std::make_unique<tf2_ros::TransformListener>(*tfBuffer);
}

// Create a normals preprocessor with the ability to transform based on normal
// orientation
NormalsPreprocessor::NormalsPreprocessor(bool debugging)
    : Preprocessor(debugging)
{
    tfBuffer = std::make_unique<tf2_ros::Buffer>();
    tfListener = std::make_unique<tf2_ros::TransformListener>(*tfBuffer);
}

// Removes points from a pointcloud (and optionally the corresponding
// pointcloud_normals as well) at given indices
void Preprocessor::removePointsFromIndices(
    const pcl::PointIndices::Ptr& indices_to_remove, const bool& remove_normals)
{
    pcl::ExtractIndices<pcl::PointXYZ> extract_points;
    extract_points.setInputCloud(pointcloud_);
    extract_points.setIndices(indices_to_remove);
    extract_points.setNegative(/*negative=*/true);
    extract_points.filter(*pointcloud_);
    if (remove_normals) {
        pcl::ExtractIndices<pcl::Normal> extract_normals;
        extract_normals.setInputCloud(pointcloud_normals_);
        extract_normals.setIndices(indices_to_remove);
        extract_normals.setNegative(/*negative=*/true);
        extract_normals.filter(*pointcloud_normals_);
    }
}

bool NormalsPreprocessor::preprocess(PointCloud::Ptr pointcloud,
    Normals::Ptr pointcloud_normals, RealSenseCategory const realsense_category,
    std::string frame_id_to_transform_to)
{
    pointcloud_ = pointcloud;
    pointcloud_normals_ = pointcloud_normals;
    frame_id_to_transform_to_ = frame_id_to_transform_to;
    realsense_category_.emplace(realsense_category);

    ROS_DEBUG_STREAM("Preprocessing with normal filtering. Pointcloud size: "
        << pointcloud_->points.size());

    clock_t start_preprocess = clock();

    bool success = true;

    success &= downsample();

    // Filter on distance before the transform to be able to find points close
    // the camera and remove them
    success &= filterOnDistanceFromOrigin();

    geometry_msgs::TransformStamped transform_stamped;
    success &= transformPointCloudFromUrdf(transform_stamped);

    // Fill normal cloud after transformation so the normals do not have to be
    // transformed
    success &= fillNormalCloud(transform_stamped);

    success &= filterOnNormalOrientation();

    clock_t end_preprocess = clock();

    if (pointcloud_->points.size() != pointcloud_normals_->points.size()) {
        ROS_ERROR_STREAM("The number of points in pointcloud and "
                         "pointcloud_normals is not equal after preprocessing. "
            << "Points in pointcloud: " << pointcloud_->points.size()
            << "Points in pointcloud_normals: "
            << pointcloud_normals_->points.size());
        return false;
    }

    double time_taken
        = double(end_preprocess - start_preprocess) / double(CLOCKS_PER_SEC);
    ROS_DEBUG_STREAM("Time taken by pointcloud pre-processor is : "
        << std::fixed << time_taken << std::setprecision(5) << " sec "
        << std::endl);

    ROS_DEBUG_STREAM("Finished preprocessing. Pointcloud size: "
        << pointcloud_->points.size());

    return success;
}

void NormalsPreprocessor::readParameters(
    march_realsense_reader::pointcloud_parametersConfig& config)
{
    // Downsampling parameters
    voxel_grid_filter = config.preprocessor_downsampling_voxel_grid_filter;
    leaf_size = (float)config.preprocessor_downsampling_leaf_size;
    random_filter = config.preprocessor_downsampling_random_filter;
    remaining_points = config.preprocessor_downsampling_remainig_points;

    // Distance Filter parameters
    maximum_distance_threshold = config.preprocessor_maximum_distance_threshold;
    minimum_distance_threshold_x
        = config.preprocessor_minimum_distance_threshold_x;
    minimum_distance_threshold_y
        = config.preprocessor_minimum_distance_threshold_y;
    minimum_distance_threshold_z
        = config.preprocessor_minimum_distance_threshold_z;

    // Normal Estimation parameters
    use_tree_search_method
        = config.preprocessor_normal_estimation_use_tree_search_method;
    number_of_neighbours
        = config.preprocessor_normal_estimation_number_of_neighbours;
    search_radius = config.preprocessor_normal_estimation_search_radius;

    // Normal filter parameters
    allowed_length_x = config.preprocessor_normal_filter_allowed_length_x;
    allowed_length_y = config.preprocessor_normal_filter_allowed_length_y;
    allowed_length_z = config.preprocessor_normal_filter_allowed_length_z;

    // Transformation parameters
    YAML::Node robot_properties
        = YAML::LoadFile(ros::package::getPath("march_description")
            + "/urdf/properties/march6.yaml");
    foot_height
        = robot_properties["dimensions"]["general"]["width"].as<double>();
    debugging_ = config.debug;
}

// Downsample the number of points in the pointcloud to have a more workable
// number of points
bool NormalsPreprocessor::downsample()
{
    // Fill in the chosen downsampling object and do the downsampling
    if (voxel_grid_filter) {
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(pointcloud_);
        voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel_grid.filter(*pointcloud_);
    } else if (random_filter) {
        pcl::RandomSample<pcl::PointXYZ> random_sampler;
        random_sampler.setInputCloud(pointcloud_);
        random_sampler.setSample(remaining_points);
        random_sampler.filter(*pointcloud_);
    }
    return true;
}

// Transform the pointcloud based on the data found on the /tf topic, this is
// necessary to know the height and distance to the wanted step from the foot.
bool NormalsPreprocessor::transformPointCloudFromUrdf(
    geometry_msgs::TransformStamped& transform_stamped)
{
    try {
        pointcloud_frame_id = pointcloud_->header.frame_id.c_str();
        if (tfBuffer->canTransform(frame_id_to_transform_to_,
                pointcloud_frame_id, ros::Time(), ros::Duration(/*t=*/1.0))) {
            transform_stamped
                = tfBuffer->lookupTransform(frame_id_to_transform_to_,
                    pointcloud_frame_id, ros::Time(/*t=*/0));
        }
        pcl_ros::transformPointCloud(
            *pointcloud_, *pointcloud_, transform_stamped.transform);
    } catch (tf2::TransformException& ex) {
        ROS_WARN_STREAM(
            "Something went wrong when transforming the pointcloud: "
            << ex.what());
        return false;
    }
    // When transforming to the foot, raise the pointcloud up a bit so that the
    // origin is on the ground If not, assume that the frame id to transform
    // chosen differently on purpose (for example to be the pressure sole which
    // is already on the ground) and do as expected.
    if (frame_id_to_transform_to_ == "foot_left"
        || frame_id_to_transform_to_ == "foot_right") {
        Eigen::Affine3f transform_matrix = Eigen::Affine3f::Identity();

        // Define a translation up of half the height of the foot.
        transform_matrix.translation() << 0, 0, foot_height / (double)2.0;

        // Executing the transformation
        pcl::transformPointCloud(*pointcloud_, *pointcloud_, transform_matrix);
    }

    return true;
}

// Remove all points which are too far or too close to the origin
bool NormalsPreprocessor::filterOnDistanceFromOrigin()
{
    pcl::PointIndices::Ptr indices_to_remove
        = boost::make_shared<pcl::PointIndices>();

    // Removed any point too far from the origin
    for (int point_index = 0; point_index < pointcloud_->points.size();
         ++point_index) {
        pcl::PointXYZ point = pointcloud_->points[point_index];

        // find the squared distance from the origin
        float point_distance = sqrt(
            (point.x * point.x) + (point.y * point.y) + (point.z * point.z));

        // remove point if it's outside the threshold distance
        if (point_distance > maximum_distance_threshold
            || (realsense_category_.value() != RealSenseCategory::sit
                && abs(point.x) < minimum_distance_threshold_x
                && abs(point.y) < minimum_distance_threshold_y
                && abs(point.z) < minimum_distance_threshold_z)) {
            indices_to_remove->indices.push_back(point_index);
        }
    }

    removePointsFromIndices(indices_to_remove, /*remove_normals=*/false);

    return true;
}

// Fill the pointcloud_normals_ object with estimated normals from the current
// pointcloud_ object The normals are oriented to the origin from before the
// transformation
bool NormalsPreprocessor::fillNormalCloud(
    const geometry_msgs::TransformStamped& transform_stamped)
{
    geometry_msgs::Vector3 translation
        = transform_stamped.transform.translation;
    //  Fill the normal estimation object and estimate the normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setInputCloud(pointcloud_);
    normal_estimator.setViewPoint(translation.x, translation.y, translation.z);

    if (use_tree_search_method) {
        pcl::search::Search<pcl::PointXYZ>::Ptr search_method(
            new pcl::search::KdTree<pcl::PointXYZ>);
        normal_estimator.setSearchMethod(search_method);
        normal_estimator.setKSearch(number_of_neighbours);
    } else {
        normal_estimator.setRadiusSearch(search_radius);
    }

    normal_estimator.compute(*pointcloud_normals_);

    return true;
}

// Filter points based on the x y or z component of the normal vector of the
// point. This can work because the normals are of unit length.
bool NormalsPreprocessor::filterOnNormalOrientation()
{
    pcl::PointIndices::Ptr indices_to_remove
        = boost::make_shared<pcl::PointIndices>();

    // Remove any point who's normal does not fall into the desired region
    if (pointcloud_->points.size() == pointcloud_normals_->points.size()) {
        for (int point_index = 0; point_index < pointcloud_->points.size();
             ++point_index) {
            // remove point if its normal is too far from what is desired
            if (pointcloud_normals_->points[point_index].normal_x
                        * pointcloud_normals_->points[point_index].normal_x
                    > allowed_length_x
                || pointcloud_normals_->points[point_index].normal_y
                        * pointcloud_normals_->points[point_index].normal_y
                    > allowed_length_y
                || pointcloud_normals_->points[point_index].normal_z
                        * pointcloud_normals_->points[point_index].normal_z
                    > allowed_length_z) {
                indices_to_remove->indices.push_back(point_index);
            }
        }
        removePointsFromIndices(indices_to_remove, /*remove_normals=*/true);
    } else {
        ROS_ERROR("The size of the pointcloud and the normal pointcloud are "
                  "not the same. Cannot filter on normals.");
        return false;
    }

    return true;
}

// Preprocess the pointcloud, this means only transforming for the simple
// preprocessor
bool SimplePreprocessor::preprocess(PointCloud::Ptr pointcloud,
    Normals::Ptr pointcloud_normals, RealSenseCategory const realsense_category,
    std::string frame_id_to_transform_to)
{
    pointcloud_ = pointcloud;
    pointcloud_normals_ = pointcloud_normals;
    frame_id_to_transform_to_ = frame_id_to_transform_to;

    ROS_DEBUG("Preprocessing with SimplePreprocessor");
    transformPointCloudFromUrdf();
    return true;
}

// Transform the pointcloud based on the data found on the /tf topic, this is
// necessary to know the height and distance to the wanted step from the foot.
void SimplePreprocessor::transformPointCloudFromUrdf()
{
    geometry_msgs::TransformStamped transformStamped;
    try {
        pointcloud_frame_id = pointcloud_->header.frame_id.c_str();
        transformStamped = tfBuffer->lookupTransform(frame_id_to_transform_to_,
            pointcloud_frame_id, ros::Time::now(), ros::Duration(/*t=*/0.5));
        pcl_ros::transformPointCloud(
            *pointcloud_, *pointcloud_, transformStamped.transform);
    } catch (tf2::TransformException& ex) {
        ROS_WARN_STREAM(
            "Something went wrong when transforming the pointcloud: "
            << ex.what());
        return;
    }
}
