#include <pointcloud_processor/preprocessor.h>

#include "yaml-cpp/yaml.h"
#include <ros/ros.h>
#include <time.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_ros/transforms.h>
#include <geometry_msgs/TransformStamped.h>


#include <pcl/filters/voxel_grid.h>


using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;

Preprocessor::Preprocessor(YAML::Node config_tree,
                           PointCloud::Ptr pointcloud,
                           Normals::Ptr pointcloud_normals):
                           config_tree_{config_tree},
                           pointcloud_{pointcloud},
                           pointcloud_normals_{pointcloud_normals}
{

}

Preprocessor::Preprocessor(
    std::string file_name,
    PointCloud::Ptr pointcloud,
    Normals::Ptr pointcloud_normals):
    pointcloud_{pointcloud},
    pointcloud_normals_{pointcloud_normals}
{
  std::string path = ros::package::getPath("march_realsense_reader") +
      "/config/" + file_name;
  config_tree_ = YAML::LoadFile(path)["preprocessor"];
}

void SimplePreprocessor::preprocess()
{
  ROS_INFO_STREAM("Preprocessing, test_parameter is " <<
  config_tree_["test_parameter"]);

}

void NormalsPreprocessor::preprocess()
{
  ROS_INFO_STREAM("Preprocessing with normal filtering. Number of points in pointcloud is " << pointcloud_->points.size());

  downsample();

  transformPointCloud();

  filterOnDistanceFromOrigin();

//  removeStatisticalOutliers();

  fillNormalCloud();

  filterOnNormalOrientation();

  ROS_INFO_STREAM("Finished preprocessing. Number of points in pointcloud is " << pointcloud_->points.size());
}

void NormalsPreprocessor::downsample()
{
  // Downsample the number of points in the pointcloud to have a more workable number of points
  auto parameters = config_tree_["downsampling"];
  double leaf_size = parameters["leaf_size"].as<double>();

  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(pointcloud_);
  voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_grid.filter(*pointcloud_);
}

void NormalsPreprocessor::removeStatisticalOutliers()
{
  // Remove statistical outliers from the pointcloud to reduce noise
  auto parameters = config_tree_["statistical_outlier_filter"];
  int number_of_neighbours = parameters["number_of_neighbours"].as<int>();
  double sd_factor = parameters["sd_factor"].as<double>();

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(pointcloud_);
  sor.setMeanK(number_of_neighbours);
  sor.setStddevMulThresh(sd_factor);
  sor.filter(*pointcloud_);
}

void NormalsPreprocessor::transformPointCloud()
{
  // Translate and rotate the pointcloud so that the origin is at the foot
  // Currently uses a very rough and static estimation of where the foot should be
  auto parameters = config_tree_["transformation"];
  double translation_x = parameters["translation_x"].as<double>();
  double translation_y = parameters["translation_y"].as<double>();
  double translation_z = parameters["translation_z"].as<double>();
  double rotation_y = parameters["rotation_y"].as<double>();

  // make a 4 by 4 transformation Transform = [Rotation (3x3) translation (3x1); 0 (1x3) 1 (1x1)]
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();

  // Add the desired translation to the transformation matrix
  transform.translation() << translation_x, translation_y, translation_z;

  // Add the desired rotation (currently just around the Y axis) to the transformation matrix
  transform.rotate(Eigen::AngleAxisf(rotation_y, Eigen::Vector3f::UnitY()));

  // Actually transform
  pcl::transformPointCloud(*pointcloud_, *pointcloud_, transform);
}

void NormalsPreprocessor::filterOnDistanceFromOrigin()
{
  // Remove all the points which are far away from the origin in 3d euclidean distance
  auto parameters = config_tree_["distance_filter"];
  double distance_threshold_squared = parameters["distance_threshold"].as<double>() *
                                      parameters["distance_threshold"].as<double>();

  for (int p = 0; p<pointcloud_->points.size(); p++)
  {
    // find the squared distance from the origin.
    float point_distance_squared = (pointcloud_->points[p].x * pointcloud_->points[p].x) +
                                   (pointcloud_->points[p].y * pointcloud_->points[p].y) +
                                   (pointcloud_->points[p].z * pointcloud_->points[p].z);

    // remove point if it's outside the threshold distance
    if (point_distance_squared > distance_threshold_squared)
    {
      pointcloud_->points[p] = pointcloud_->points[pointcloud_->points.size() - 1];
      pointcloud_->points.resize(pointcloud_->points.size() - 1);

      p--;
    }
  }
}

void NormalsPreprocessor::fillNormalCloud()
{
  // Fill the pointcloud_normals_ object with estimated normals from the current pointcloud_ object 
  auto parameters = config_tree_["normal_estimation"];
  bool use_tree_search_method = parameters["use_tree_search_method"].as<bool>();
  int number_of_neighbours = parameters["number_of_neighbours"].as<int>();
  double search_radius = parameters["search_radius"].as<double>();

  auto transformation_parameters = config_tree_["transformation"];
  double translation_x = transformation_parameters["translation_x"].as<double>();
  double translation_y = transformation_parameters["translation_y"].as<double>();
  double translation_z = transformation_parameters["translation_z"].as<double>();

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

void NormalsPreprocessor::filterOnNormalOrientation()
{
  // Filter points based on the x y or z component of the normal vector of the point.
  // This can work because the normals are of unit length.
  auto parameters = config_tree_["normal_filter"];
  double allowed_length_x = parameters["allowed_length_x"].as<double>();
  double allowed_length_y = parameters["allowed_length_y"].as<double>();
  double allowed_length_z = parameters["allowed_length_z"].as<double>();

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
          allowed_length_z )
      {
        pointcloud_->points[p] = pointcloud_->points[pointcloud_->points.size() - 1];
        pointcloud_->points.resize(pointcloud_->points.size() - 1);

        pointcloud_normals_->points[p] = pointcloud_normals_->points[pointcloud_normals_->points.size() - 1];
        pointcloud_normals_->points.resize(pointcloud_normals_->points.size() - 1);

        p--;
      }
    }
  }
  else
  {
    ROS_WARN("The size of the pointcloud and the normal pointcloud are not the same. Cannot filter on normals.");
  }
}
