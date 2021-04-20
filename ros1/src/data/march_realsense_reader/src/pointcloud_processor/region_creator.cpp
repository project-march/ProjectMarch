#include "pointcloud_processor/region_creator.h"
#include "yaml-cpp/yaml.h"
#include <ctime>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <ros/ros.h>
#include <utilities/yaml_utilities.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using ColoredPointCloud = pcl::PointCloud<pcl::PointXYZRGB>;
using Normals = pcl::PointCloud<pcl::Normal>;
using RegionVector = std::vector<pcl::PointIndices>;

// Construct a basic RegionCreator class
RegionCreator::RegionCreator(YAML::Node config_tree, bool debugging)
    : config_tree_ { config_tree }
    , debugging_ { debugging }
{
}

// Construct a basic RegionGrower class
RegionGrower::RegionGrower(YAML::Node config_tree, bool debugging)
    : RegionCreator(config_tree, debugging)
{
    readYaml();
}

bool RegionGrower::createRegions(PointCloud::Ptr pointcloud,
    Normals::Ptr pointcloud_normals,
    boost::shared_ptr<RegionVector> region_vector)
{
    pointcloud_ = pointcloud;
    pointcloud_normals_ = pointcloud_normals;
    region_vector_ = region_vector;
    ROS_DEBUG_STREAM("Creating regions with region growing");

    clock_t start_region_grow = clock();

    bool success = true;
    success &= setupRegionGrower();
    success &= extractRegions();

    clock_t end_region_grow = clock();
    double time_taken
        = double(end_region_grow - start_region_grow) / double(CLOCKS_PER_SEC);
    ROS_DEBUG_STREAM("Time taken by pointcloud RegionGrower is : "
        << std::fixed << time_taken << std::setprecision(5) << " sec "
        << std::endl);

    return success;
}

void RegionGrower::readYaml()
{
    min_cluster_size
        = yaml_utilities::grabParameter<int>(config_tree_, "min_cluster_size");
    max_cluster_size
        = yaml_utilities::grabParameter<int>(config_tree_, "max_cluster_size");
    if (YAML::Node region_growing_parameters = config_tree_["region_growing"]) {
        number_of_neighbours = yaml_utilities::grabParameter<int>(
            region_growing_parameters, "number_of_neighbours");
        smoothness_threshold = yaml_utilities::grabParameter<float>(
            region_growing_parameters, "smoothness_threshold");
        curvature_threshold = yaml_utilities::grabParameter<float>(
            region_growing_parameters, "curvature_threshold");
    } else {
        ROS_ERROR("'region_growing' parameters not found in parameter file");
    }
}

bool RegionGrower::setupRegionGrower()
{
    if (pointcloud_->size() == pointcloud_normals_->size()) {
        pcl::search::Search<pcl::PointXYZ>::Ptr tree(
            new pcl::search::KdTree<pcl::PointXYZ>);
        region_grower.setMinClusterSize(min_cluster_size);
        region_grower.setMaxClusterSize(max_cluster_size);
        region_grower.setSearchMethod(tree);
        region_grower.setNumberOfNeighbours(number_of_neighbours);
        region_grower.setInputCloud(pointcloud_);
        region_grower.setInputNormals(pointcloud_normals_);
        region_grower.setSmoothnessThreshold(smoothness_threshold);
        region_grower.setCurvatureThreshold(curvature_threshold);
        return true;
    } else {
        ROS_ERROR("pointcloud_ is of size: %lu, while pointcloud_normals_ is "
                  "of size: %lu",
            pointcloud_->size(), pointcloud_normals_->size());
        return false;
    }
}

bool RegionGrower::extractRegions()
{
    region_grower.extract(*region_vector_);
    if (debugging_) {
        ROS_DEBUG(
            "Total number of clusters found: %lu", region_vector_->size());
        int i = 0;
        for (auto region : *region_vector_) {
            ROS_DEBUG("Total number of points in cluster %i: %lu", i,
                region.indices.size());
            i++;
        }

        if (region_vector_->size() == 0) {
            ROS_WARN("Region growing algorithm found no clusters");
            return false;
        }
        return true;
    }

    ROS_ERROR("Something went wrong during extracting the regions from the "
              "region grower.");
    return false;
}

ColoredPointCloud::Ptr RegionGrower::debug_visualisation()
{
    return region_grower.getColoredCloud();
}

// Contrust a basic EuclideanClustering class
EuclideanClustering::EuclideanClustering(YAML::Node config_tree, bool debugging)
    : RegionCreator(config_tree, debugging)
{
    readYaml();
}

void EuclideanClustering::readYaml()
{
    min_cluster_size
        = yaml_utilities::grabParameter<int>(config_tree_, "min_cluster_size");
    max_cluster_size
        = yaml_utilities::grabParameter<int>(config_tree_, "max_cluster_size");
    if (YAML::Node euclidean_clustering_parameters
        = config_tree_["euclidean_clustering"]) {
        distance_tolerance = yaml_utilities::grabParameter<double>(
            euclidean_clustering_parameters, "distance_tolerance");
    }
}

bool EuclideanClustering::createRegions(PointCloud::Ptr pointcloud,
                                        Normals::Ptr pointcloud_normals,
                                        boost::shared_ptr<RegionVector> region_vector)
{
    pointcloud_ = pointcloud;
    pointcloud_normals_ = pointcloud_normals;
    region_vector_ = region_vector;
    ROS_DEBUG_STREAM("Creating regions with region growing");

    clock_t start_region_grow = clock();

    bool success = true;
    success &= createEuclideanClusters();

    clock_t end_region_grow = clock();
    double time_taken
            = double(end_region_grow - start_region_grow) / double(CLOCKS_PER_SEC);
    ROS_DEBUG_STREAM("Time taken by pointcloud RegionGrower is : "
                             << std::fixed << time_taken << std::setprecision(5) << " sec "
                             << std::endl);

    return success;
}

bool EuclideanClustering::createEuclideanClusters()
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(pointcloud_);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidean_clusterer;

    euclidean_clusterer.setClusterTolerance(distance_tolerance);
    euclidean_clusterer.setMinClusterSize(min_cluster_size);
    euclidean_clusterer.setMaxClusterSize(max_cluster_size);
    euclidean_clusterer.setSearchMethod(tree);
    euclidean_clusterer.setInputCloud(pointcloud_);
    euclidean_clusterer.extract(*region_vector_);

    ROS_DEBUG("Total number of clusters found: %lu", region_vector_->size());
    for (int cluster_index = 0; cluster_index < region_vector_->size(); cluster_index++) {
        ROS_DEBUG("Total number of points in cluster %i: %lu", cluster_index,
                  region_vector_->at(cluster_index).indices.size());
    }

    if (region_vector_->size() == 0) {
        ROS_WARN("Region growing algorithm found no clusters");
        return false;
    }
    return true;
}

ColoredPointCloud::Ptr EuclideanClustering::debug_visualisation()
{
    ColoredPointCloud::Ptr colored_cloud = boost::make_shared<ColoredPointCloud>();
    // Initialize the object with which to get the cloud corresponding to the
    // indices of the clusters
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    for (pcl::PointIndices region : *region_vector_) {
        PointCloud::Ptr region_cloud = boost::make_shared<PointCloud>();
        ColoredPointCloud::Ptr colored_region
            = boost::make_shared<ColoredPointCloud>();

        // Extract the region from the point cloud
        pcl::copyPointCloud(*pointcloud_, region, *region_cloud);

        int r = rand() % 256;
        int g = rand() % 256;
        int b = rand() % 256;

        int region_size = region.indices.size();
        colored_region->points.resize(region_size);

        for (int point_index = 0; point_index < region_size; point_index++) {
            colored_region->points[point_index].x
                = region_cloud->points[point_index].x;
            colored_region->points[point_index].y
                = region_cloud->points[point_index].y;
            colored_region->points[point_index].z
                = region_cloud->points[point_index].z;

            colored_region->points[point_index].r = r;
            colored_region->points[point_index].g = g;
            colored_region->points[point_index].b = b;
        }
        *colored_cloud += *colored_region;
    }
    return colored_cloud;
}