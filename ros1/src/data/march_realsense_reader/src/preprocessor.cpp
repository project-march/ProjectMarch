#include <pointcloud_processor/preprocessor.h>
#include "yaml-cpp/yaml.h"
#include <ros/ros.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>


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
  ROS_INFO_STREAM("Preprocessing with normal filtering.");
  ROS_INFO_STREAM("Number of points BEFORE preprocessor " << pointcloud_->points.size());
  ROS_INFO_STREAM("Number of normals BEFORE preprocessor " << pointcloud_normals_->points.size());

  downsample();
  removeStatisticalOutliers();
  transformPointCloud();
  filterOnDistanceFromOrigin();
  fillNormalCloud();
  filterOnNormalOrientation();

  ROS_INFO_STREAM("Number of points AFTER preprocessor " << pointcloud_->points.size());
  ROS_INFO_STREAM("Number of normals AFTER preprocessor " << pointcloud_normals_->points.size());
}

void NormalsPreprocessor::downsample()
{
  // Downsample the number of points in the pointcloud to have a more workable number of points
  auto parameters = config_tree_["downsampling"];

  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud (pointcloud_);
  voxel_grid.setLeafSize (parameters["leaf_size"].as<double>(),
                          parameters["leaf_size"].as<double>(),
                          parameters["leaf_size"].as<double>());
  voxel_grid.filter (*pointcloud_);
}

void NormalsPreprocessor::removeStatisticalOutliers()
{
  // Remove statistical outliers from the pointcloud to reduce noise
  auto parameters = config_tree_["statistical_outlier_filter"];

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (pointcloud_);
  sor.setMeanK (parameters["number_of_neighbours"].as<int>());
  sor.setStddevMulThresh (parameters["sd_factor"].as<double>());
  sor.filter (*pointcloud_);
}

void NormalsPreprocessor::transformPointCloud()
{
  // Translate and rotate the pointcloud so that the origin is at the foot
  // Currently uses a very rough and static estimation of where the foot should be
  auto parameters = config_tree_["transformation"];

  Eigen::Affine3f transform = Eigen::Affine3f::Identity(); // Eigen should be a dependency of pcl, is this fine?
  // Add the desired translation to the transformation matrix
  transform.translation() << parameters["translation_x"].as<double>(),
                             parameters["translation_y"].as<double>(),
                             parameters["translation_z"].as<double>();
  // Add the desired rotation (currently just around the Y axis) to the transformation matrix
  transform.rotate(Eigen::AngleAxisf(parameters["rotation_y"].as<double>(),
                                     Eigen::Vector3f::UnitY()));
  pcl::transformPointCloud(*pointcloud_, *pointcloud_, transform); // Actually transform
}

void NormalsPreprocessor::filterOnDistanceFromOrigin()
{
  // Remove all the points which are far away from the origin in 3d euclidean distance
  auto parameters = config_tree_["distance_filter"];

  for (int p = 0; p<pointcloud_->points.size(); p++)
  {
    // find the squared distance from the origin.
    float point_distance_squared = (pointcloud_->points[p].x * pointcloud_->points[p].x) +
                                   (pointcloud_->points[p].y * pointcloud_->points[p].y) +
                                   (pointcloud_->points[p].z * pointcloud_->points[p].z);

    // remove point if it's outside the threshold distance
    if (point_distance_squared > parameters["distance_threshold_squared"].as<double>())
    {
      pointcloud_->points[p] = pointcloud_->points[pointcloud_->points.size() - 1];
      pointcloud_->points.resize(pointcloud_->points.size() - 1);

      p--;
    }
  }
}

void NormalsPreprocessor::fillNormalCloud()
{
  // Remove all the points who's normal is not in the 'interesting' region.
  auto parameters = config_tree_["normal_estimation"];

  pcl::NormalEstimation <pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setInputCloud(pointcloud_);
  if (parameters["use_tree_search_method"].as<bool>())
  {
    pcl::search::Search<pcl::PointXYZ>::Ptr search_method (new pcl::search::KdTree <pcl::PointXYZ>);
    normal_estimator.setSearchMethod(search_method);
    normal_estimator.setKSearch(parameters["number_of_neighbours"].as<int>());
  }
  else
  {
    normal_estimator.setRadiusSearch(parameters["search_radius"].as<double>());
  }
  normal_estimator.compute(*pointcloud_normals_);
}

void NormalsPreprocessor::filterOnNormalOrientation()
{
  auto parameters = config_tree_["normal_filter"];

  if (pointcloud_->points.size() == pointcloud_normals_->points.size())
  {
    for (int p = 0; p < pointcloud_->points.size(); p++)
    {
      // remove point if its normal is too far from what is desired
      if (pointcloud_normals_->points[p].normal_x * pointcloud_normals_->points[p].normal_x >
          parameters["allowed_length_x"].as<double>() ||
          pointcloud_normals_->points[p].normal_y * pointcloud_normals_->points[p].normal_y >
          parameters["allowed_length_y"].as<double>() ||
          pointcloud_normals_->points[p].normal_z * pointcloud_normals_->points[p].normal_z >
          parameters["allowed_length_z"].as<double>() )
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