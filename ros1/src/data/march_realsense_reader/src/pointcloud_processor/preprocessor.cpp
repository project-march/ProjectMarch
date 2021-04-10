#include <pointcloud_processor/preprocessor.h>
#include <utilities/yaml_utilities.h>
#include "yaml-cpp/yaml.h"
#include <ros/ros.h>
#include <ctime>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_listener.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;

// Base constructor for preprocessors
Preprocessor::Preprocessor(YAML::Node config_tree, bool debugging):
                           config_tree_{config_tree},
                           debugging_{debugging}
{

}

// Create a simple preprocessor with the ability to look up transforms
SimplePreprocessor::SimplePreprocessor(YAML::Node config_tree, bool debugging):
    Preprocessor(config_tree, debugging)
{
  tfBuffer = std::make_unique<tf2_ros::Buffer>();
  tfListener = std::make_unique<tf2_ros::TransformListener>(*tfBuffer);
}

// Create a normals preprocessor with the ability to transform based on normal orientation
NormalsPreprocessor::NormalsPreprocessor(YAML::Node config_tree, bool debugging):
        Preprocessor(config_tree, debugging)
{
  tfBuffer = std::make_unique<tf2_ros::Buffer>();
  tfListener = std::make_unique<tf2_ros::TransformListener>(*tfBuffer);
  readYaml();
}

// Removes a point from a pointcloud (and optionally the corresponding pointcloud_normals as well) at a given index
void Preprocessor::removePointByIndex(int const index, PointCloud::Ptr pointcloud, Normals::Ptr pointcloud_normals)
{
  if (index < pointcloud->points.size() && index >= 0)
  {
    if (pointcloud_normals != nullptr)
    {
      if (index < pointcloud_normals->points.size() && index >= 0)
      {
        pointcloud_normals->points[index] = pointcloud_normals->points[pointcloud_normals->points.size() - 1];
        pointcloud_normals->points.resize(pointcloud_normals->points.size() - 1);
      }
      else
      {
        ROS_WARN_STREAM("Index " << index << " to be removed is not valid for pointcloud_normals");
      }
    }
    pointcloud->points[index] = pointcloud->points[pointcloud->points.size() - 1];
    pointcloud->points.resize(pointcloud->points.size() - 1);
  }
  else
  {
    ROS_WARN_STREAM("Index " << index << " to be removed is not valid for pointcloud");
  }
}

bool NormalsPreprocessor::preprocess(
    PointCloud::Ptr pointcloud, Normals::Ptr pointcloud_normals, std::string frame_id_to_transform_to)
{
  pointcloud_ = pointcloud;
  pointcloud_normals_ = pointcloud_normals;
  frame_id_to_transform_to_ = frame_id_to_transform_to;

  ROS_DEBUG_STREAM("Preprocessing with normal filtering. Pointcloud size: " << pointcloud_->points.size());

  clock_t start_preprocess = clock();

  bool success = true;

  success &= downsample();

  geometry_msgs::TransformStamped transform_stamped;
  success &= transformPointCloudFromUrdf(transform_stamped);

  success &= filterOnDistanceFromOrigin();

  success &= fillNormalCloud(transform_stamped);

  success &= filterOnNormalOrientation();

  clock_t end_preprocess = clock();

  if (pointcloud_->points.size() != pointcloud_normals_->points.size())
  {
    ROS_ERROR_STREAM("The number of points in pointcloud and pointcloud_normals is not equal after preprocessing. "
                    << "Points in pointcloud: " << pointcloud_->points.size()
                    << "Points in pointcloud_normals: " << pointcloud_normals_->points.size());
  }

  double time_taken = double(end_preprocess - start_preprocess) / double(CLOCKS_PER_SEC);
  ROS_DEBUG_STREAM("Time taken by pointcloud pre-processor is : " << std::fixed <<
                   time_taken << std::setprecision(5) << " sec " << std::endl);

  ROS_DEBUG_STREAM("Finished preprocessing. Pointcloud size: " << pointcloud_->points.size());

  return success;
}

void NormalsPreprocessor::readYaml()
{
  getDownsamplingParameters();

  getDistanceFilterParameters();

  getNormalEstimationParameters();

  getNormalFilterParameters();
}

void NormalsPreprocessor::getDownsamplingParameters()
{
  // Grab downsampling parameters
  if (YAML::Node downsampling_parameters = config_tree_["downsampling"])
  {
    voxel_grid_filter = yaml_utilities::grabParameter<bool>(downsampling_parameters, "voxel_grid_filter");
    random_filter = yaml_utilities::grabParameter<bool>(downsampling_parameters, "random_filter");
    if (voxel_grid_filter)
    {
      leaf_size = yaml_utilities::grabParameter<double>(downsampling_parameters, "leaf_size");
    }
    else if (random_filter)
    {
      remaining_points = yaml_utilities::grabParameter<int>(downsampling_parameters, "remaining_points");
    }
    else
    {
      ROS_WARN_STREAM("No downsampling method was selected. Continuing without downsampling.");
    }
  }
  else
  {
    ROS_ERROR("Downsample parameters not found in parameter file");
  }
}

void NormalsPreprocessor::getDistanceFilterParameters()
{
  //  Grab distance filter parameters
  if (YAML::Node parameters = config_tree_["distance_filter"])
  {
    distance_threshold = yaml_utilities::grabParameter<double>(parameters, "distance_threshold");
  }
  else
  {
    ROS_ERROR("Distance filter parameters not found in parameter file");
  }
}

void NormalsPreprocessor::getNormalEstimationParameters()
{
  //  Grab normal estimation parameters
  if (YAML::Node normal_estimation_parameters = config_tree_["normal_estimation"])
  {
    use_tree_search_method = yaml_utilities::grabParameter<bool>(normal_estimation_parameters, "use_tree_search_method");
    if (use_tree_search_method)
    {
      number_of_neighbours = yaml_utilities::grabParameter<int>(normal_estimation_parameters, "number_of_neighbours");
    }
    else
    {
      search_radius = yaml_utilities::grabParameter<double>(normal_estimation_parameters, "search_radius");
    }
  }
  else
  {
    ROS_ERROR("Normal estimation parameters not found in parameter file");
  }
}

void NormalsPreprocessor::getNormalFilterParameters()
{
  //  Grab normal filter parameters
  if (YAML::Node normal_filter_parameters = config_tree_["normal_filter"])
  {
    allowed_length_x = yaml_utilities::grabParameter<double>(normal_filter_parameters, "allowed_length_x");
    allowed_length_y = yaml_utilities::grabParameter<double>(normal_filter_parameters, "allowed_length_y");
    allowed_length_z = yaml_utilities::grabParameter<double>(normal_filter_parameters, "allowed_length_z");
  }
  else
  {
    ROS_ERROR("Normal filter parameters not found in parameter file");
  }
}

// Downsample the number of points in the pointcloud to have a more workable number of points
bool NormalsPreprocessor::downsample()
{
  // Fill in the chosen downsampling object and do the downsampling
  if (voxel_grid_filter)
  {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(pointcloud_);
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.filter(*pointcloud_);
  }
  else if (random_filter)
  {
    pcl::RandomSample<pcl::PointXYZ> random_sampler;
    random_sampler.setInputCloud(pointcloud_);
    random_sampler.setSample(remaining_points);
    random_sampler.filter(*pointcloud_);
  }
  return true;
}

// Transform the pointcloud based on the data found on the /tf topic, this is
// necessary to know the height and distance to the wanted step from the foot.
bool NormalsPreprocessor::transformPointCloudFromUrdf(geometry_msgs::TransformStamped & transform_stamped)
{
  try
  {
    pointcloud_frame_id = pointcloud_->header.frame_id.c_str();
    transform_stamped = tfBuffer->lookupTransform(frame_id_to_transform_to_, pointcloud_frame_id,
                                                 ros::Time::now(), ros::Duration(0.5));
    pcl_ros::transformPointCloud(*pointcloud_, *pointcloud_,
                                 transform_stamped.transform);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_STREAM("Something went wrong when transforming the pointcloud: "
                            << ex.what());
    return false;
  }
  return true;
}

// Translate and rotate the pointcloud so that the origin is at the foot
// Currently uses a very rough and static estimation of where the foot should be
bool NormalsPreprocessor::transformPointCloud()
{
  // make a 4 by 4 transformation Transform = [Rotation (3x3) translation (3x1); 0 (1x3) 1 (1x1)]
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();

  // Add the desired rotation (currently just around the Y axis) to the transformation matrix
  transform.rotate(Eigen::AngleAxisf(rotation_y, Eigen::Vector3f::UnitY()));

  // Actually transform
  pcl::transformPointCloud(*pointcloud_, *pointcloud_, transform);

  return true;
}

// Remove all the points which are far away from the origin in 3d euclidean distance
bool NormalsPreprocessor::filterOnDistanceFromOrigin()
{
  double distance_threshold_squared = distance_threshold * distance_threshold;

  // Removed any point too far from the origin
  for (int p = 0; p<pointcloud_->points.size(); p++)
  {
    // find the squared distance from the origin
    float point_distance_squared = (pointcloud_->points[p].x * pointcloud_->points[p].x) +
                                   (pointcloud_->points[p].y * pointcloud_->points[p].y) +
                                   (pointcloud_->points[p].z * pointcloud_->points[p].z);

    // remove point if it's outside the threshold distance
    if (point_distance_squared > distance_threshold_squared)
    {
      removePointByIndex(p, pointcloud_);
      p--;
    }
  }
  return true;
}

// Fill the pointcloud_normals_ object with estimated normals from the current pointcloud_ object
// The normals are oriented to the origin from before the transformation
bool NormalsPreprocessor::fillNormalCloud(geometry_msgs::TransformStamped transform_stamped)
{
  geometry_msgs::Vector3 translation = transform_stamped.transform.translation;
  //  Fill the normal estimation object and estimate the normals
  pcl::NormalEstimation <pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setInputCloud(pointcloud_);
  normal_estimator.setViewPoint(translation.x, translation.y, translation.z);

  if (use_tree_search_method)
  {
    pcl::search::Search<pcl::PointXYZ>::Ptr search_method (new pcl::search::KdTree <pcl::PointXYZ>);
    normal_estimator.setSearchMethod(search_method);
    normal_estimator.setKSearch(number_of_neighbours);
  }
  else
  {
    normal_estimator.setRadiusSearch(search_radius);
  }
  normal_estimator.compute(*pointcloud_normals_);

  return true;
}

// Filter points based on the x y or z component of the normal vector of the point.
// This can work because the normals are of unit length.
bool NormalsPreprocessor::filterOnNormalOrientation()
{
  // Remove any point who's normal does not fall into the desired region
  if (pointcloud_->points.size() == pointcloud_normals_->points.size())
  {
    for (int p = 0; p < pointcloud_->points.size(); p++)
    {
      // remove point if its normal is too far from what is desired
      if (pointcloud_normals_->points[p].normal_x * pointcloud_normals_->points[p].normal_x >
          allowed_length_x ||
          pointcloud_normals_->points[p].normal_y * pointcloud_normals_->points[p].normal_y >
          allowed_length_y ||
          pointcloud_normals_->points[p].normal_z * pointcloud_normals_->points[p].normal_z >
          allowed_length_z)
      {
        removePointByIndex(p, pointcloud_, pointcloud_normals_);
        p--;
      }
    }
  }
  else
  {
    ROS_ERROR("The size of the pointcloud and the normal pointcloud are not the same. Cannot filter on normals.");
    return false;
  }

  return true;
}

// Preprocess the pointcloud, this means only transforming for the simple preprocessor
bool SimplePreprocessor::preprocess(PointCloud::Ptr pointcloud,
                                    Normals::Ptr pointcloud_normals,
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
  try
  {
    pointcloud_frame_id = pointcloud_->header.frame_id.c_str();
    transformStamped = tfBuffer->lookupTransform(frame_id_to_transform_to_, pointcloud_frame_id,
                                                  ros::Time::now(), ros::Duration(0.5));
    pcl_ros::transformPointCloud(*pointcloud_, *pointcloud_,
                                 transformStamped.transform);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_STREAM("Something went wrong when transforming the pointcloud: "
                        << ex.what());
    return;
  }
}
