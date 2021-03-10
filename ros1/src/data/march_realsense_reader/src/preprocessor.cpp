#include <pointcloud_processor/preprocessor.h>
#include <yaml_utilities.h>
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
Preprocessor::Preprocessor(YAML::Node config_tree):
                           config_tree_{config_tree}
{

}

// Create a simple preprocessor with the ability to look up transforms
SimplePreprocessor::SimplePreprocessor(YAML::Node config_tree):
    Preprocessor(config_tree)
{
  tfBuffer = std::make_unique<tf2_ros::Buffer>();
  tfListener = std::make_unique<tf2_ros::TransformListener>(*tfBuffer);
}

// Create a preprocessor with the ability to estimate normals and filter based on them
NormalsPreprocessor::NormalsPreprocessor(YAML::Node config_tree):
        Preprocessor(config_tree)
{

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

void NormalsPreprocessor::preprocess(PointCloud::Ptr pointcloud, Normals::Ptr pointcloud_normals)
{
  pointcloud_ = pointcloud;
  pointcloud_normals_ = pointcloud_normals;

  ROS_INFO_STREAM("Preprocessing with normal filtering. Pointcloud size: " << pointcloud_->points.size());

  bool do_statistical_outlier_removal = false;
  if (YAML::Node statistical_outlier_filter_parameters = config_tree_["statistical_outlier_filter"])
  {
    do_statistical_outlier_removal = yaml_utilities::grabParameter<bool>(statistical_outlier_filter_parameters,
                                                                         "do_statistical_outlier_removal");
  }
  clock_t start = clock();

  downsample();
  clock_t downsample = clock();

  transformPointCloud();
  clock_t transformPointCloud = clock();

  filterOnDistanceFromOrigin();
  clock_t filterOnDistanceFromOrigin = clock();

  fillNormalCloud();
  clock_t fillNormalCloud = clock();

  filterOnNormalOrientation();

  int points_before = pointcloud_->points.size();
  clock_t filterOnNormalOrientation = clock();
  if (do_statistical_outlier_removal)
  {
    removeStatisticalOutliers();
  }
  clock_t removeStatisticalOutliers = clock();
  int points_after = pointcloud_->points.size();

  if (pointcloud_->points.size() != pointcloud_normals_->points.size())
  {
    ROS_ERROR_STREAM("The number of points in pointcloud and pointcloud_normals is not equal after preprocessing. "
                    << "Points in pointcloud: " << pointcloud_->points.size()
                    << "Points in pointcloud_normals: " << pointcloud_normals_->points.size());
  }


  ROS_INFO_STREAM("Finished preprocessing. Pointcloud size: " << pointcloud_->points.size());
  double time_taken = double(downsample - start) / double(CLOCKS_PER_SEC);
  std::cout << "Time taken by downsample is : " << std::fixed
            << time_taken << std::setprecision(5);
  std::cout << " sec " << std::endl;
  time_taken = double(transformPointCloud - downsample) / double(CLOCKS_PER_SEC);
  std::cout << "Time taken by transformPointCloud is : " << std::fixed
            << time_taken << std::setprecision(5);
  std::cout << " sec " << std::endl;
  time_taken = double(filterOnDistanceFromOrigin - transformPointCloud) / double(CLOCKS_PER_SEC);
  std::cout << "Time taken by filterOnDistanceFromOrigin is : " << std::fixed
            << time_taken << std::setprecision(5);
  std::cout << " sec " << std::endl;
  time_taken = double(fillNormalCloud - filterOnDistanceFromOrigin) / double(CLOCKS_PER_SEC);
  std::cout << "Time taken by fillNormalCloud is : " << std::fixed
            << time_taken << std::setprecision(5);
  std::cout << " sec " << std::endl;
  time_taken = double(filterOnNormalOrientation - fillNormalCloud) / double(CLOCKS_PER_SEC);
  std::cout << "Time taken by filterOnNormalOrientation is : " << std::fixed
            << time_taken << std::setprecision(5);
  std::cout << " sec " << std::endl;
  time_taken = double(removeStatisticalOutliers - filterOnNormalOrientation) / double(CLOCKS_PER_SEC);
  std::cout << "Time taken by removeStatisticalOutliers is : " << std::fixed
       << time_taken << std::setprecision(5);
  std::cout << " sec " << std::endl;

  double removed_SOR = double(points_after - points_before);
  std::cout << "Points removed by SOR: " << removed_SOR << std::endl;

  time_taken = double(removeStatisticalOutliers - start) / double(CLOCKS_PER_SEC);
  std::cout << "Time taken by program is : " << std::fixed
            << time_taken << std::setprecision(5);
  std::cout << " sec " << std::endl;
}

// Downsample the number of points in the pointcloud to have a more workable number of points
void NormalsPreprocessor::downsample()
{
  //  Grab relevant parameters
  bool voxel_grid_filter = false;
  double leaf_size;
  bool random_filter = false;
  int remaining_points;
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

  // Fill in the chosen downsampling object and do the downsampling
  if (voxel_grid_filter)
  {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(pointcloud_);
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.setFilterLimits(-3.0, 3.0);
    voxel_grid.filter(*pointcloud_);
  }
  else if (random_filter)
  {
    pcl::RandomSample<pcl::PointXYZ> random_sampler;
    random_sampler.setInputCloud(pointcloud_);
    random_sampler.setSample(remaining_points);
    random_sampler.filter(*pointcloud_);
  }
}

// Translate and rotate the pointcloud so that the origin is at the foot
// Currently uses a very rough and static estimation of where the foot should be
void NormalsPreprocessor::transformPointCloud()
{
  //  Grab relevant parameters
  double translation_x;
  double translation_y;
  double translation_z;
  double rotation_y;
  if (YAML::Node transformation_parameters = config_tree_["transformation"])
  {
    translation_x = yaml_utilities::grabParameter<double>(transformation_parameters, "translation_x");
    translation_y = yaml_utilities::grabParameter<double>(transformation_parameters, "translation_y");
    translation_z = yaml_utilities::grabParameter<double>(transformation_parameters, "translation_z");
    rotation_y = yaml_utilities::grabParameter<double>(transformation_parameters, "rotation_y");
  }
  else
  {
    ROS_ERROR("Transformation parameters not found in parameter file");
  }

  // make a 4 by 4 transformation Transform = [Rotation (3x3) translation (3x1); 0 (1x3) 1 (1x1)]
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();

  // Add the desired translation to the transformation matrix
  transform.translation() << translation_x, translation_y, translation_z;

  // Add the desired rotation (currently just around the Y axis) to the transformation matrix
  transform.rotate(Eigen::AngleAxisf(rotation_y, Eigen::Vector3f::UnitY()));

  // Actually transform
  pcl::transformPointCloud(*pointcloud_, *pointcloud_, transform);
}

// Remove all the points which are far away from the origin in 3d euclidean distance
void NormalsPreprocessor::filterOnDistanceFromOrigin()
{
  //  Grab relevant parameters
  double distance_threshold;
  if (YAML::Node parameters = config_tree_["distance_filter"])
  {
    distance_threshold = yaml_utilities::grabParameter<double>(parameters, "distance_threshold");
  }
  else
  {
    ROS_ERROR("Distance filter parameters not found in parameter file");
  }
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
}

// Fill the pointcloud_normals_ object with estimated normals from the current pointcloud_ object
void NormalsPreprocessor::fillNormalCloud()
{
  //  Grab relevant parameters
  double translation_x;
  double translation_y;
  double translation_z;
  if (YAML::Node transformation_parameters = config_tree_["transformation"])
  {
    translation_x = yaml_utilities::grabParameter<double>(transformation_parameters, "translation_x");
    translation_y = yaml_utilities::grabParameter<double>(transformation_parameters, "translation_y");
    translation_z = yaml_utilities::grabParameter<double>(transformation_parameters, "translation_z");
  }
  else
  {
    ROS_ERROR("Transformation parameters not found in parameter file");
  }

  bool use_tree_search_method;
  int number_of_neighbours;
  double search_radius;
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

  //  Fill the normal estimation object and estimate the normals
  pcl::NormalEstimation <pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setInputCloud(pointcloud_);
  normal_estimator.setViewPoint(translation_x, translation_y, translation_z);

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
}

// Filter points based on the x y or z component of the normal vector of the point.
// This can work because the normals are of unit length.
void NormalsPreprocessor::filterOnNormalOrientation()
{
  //  Grab relevant parameters
  double allowed_length_x;
  double allowed_length_y;
  double allowed_length_z;
  if (YAML::Node normal_filter_parameters = config_tree_["normal_filter"])
  {
    allowed_length_x = yaml_utilities::grabParameter<double>(normal_filter_parameters, "allowed_length_x");
    allowed_length_y = yaml_utilities::grabParameter<double>(normal_filter_parameters, "allowed_length_y");
    allowed_length_z = yaml_utilities::grabParameter<double>(normal_filter_parameters, "allowed_length_z");
  }

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
  }
}

// Remove statistical outliers from the pointcloud to reduce noise
void NormalsPreprocessor::removeStatisticalOutliers()
{
  //  Grab relevant parameters
  int number_of_neighbours;
  double sd_factor;
  if (YAML::Node statistical_outlier_removal_parameters = config_tree_["statistical_outlier_removal"])
  {
    number_of_neighbours = yaml_utilities::grabParameter<int>(statistical_outlier_removal_parameters,
                                                              "number_of_neighbours");
    sd_factor = yaml_utilities::grabParameter<double>(statistical_outlier_removal_parameters, "sd_factor");
  }

  // Fill the SOR object and execute the filter
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(pointcloud_);
  sor.setMeanK(number_of_neighbours);
  sor.setStddevMulThresh(sd_factor);
  sor.filter(*pointcloud_);
}

// Preprocess the pointcloud, this means only transforming for the simple preprocessor
void SimplePreprocessor::preprocess(PointCloud::Ptr pointcloud,
                                    Normals::Ptr pointcloud_normals)
{
  pointcloud_ = pointcloud;
  pointcloud_normals_ = pointcloud_normals;

  int test_parameter;
  test_parameter = yaml_utilities::grabParameter<int>(config_tree_, "test_parameter");

  ROS_INFO_STREAM("Preprocessing with simple preprocessor. Test parameter is " << test_parameter);

  transformPointCloudFromUrdf();
}

// Transform the pointcloud based on the data found on the /tf topic, this is
// necessary to know the height and distance to the wanted step from the foot.
void SimplePreprocessor::transformPointCloudFromUrdf()
{
  geometry_msgs::TransformStamped transformStamped;
  try
  {
    transformStamped = tfBuffer->lookupTransform("camera_link", "foot_left",
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
