#include "pointcloud_processor/region_creator.h"
#include <ctime>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <ros/ros.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;
using RegionVector = std::vector<pcl::PointIndices>;

// Construct a basic RegionCreator class
RegionCreator::RegionCreator(bool debugging)
    : debugging_ { debugging }
{
}

// Construct a basic CHullFinder class
RegionGrower::RegionGrower(bool debugging)
    : RegionCreator(debugging)
    , number_of_neighbours(-1)
    , min_cluster_size(-1)
    , max_cluster_size(-1)
    , smoothness_threshold(std::numeric_limits<float>::lowest())
    , curvature_threshold(std::numeric_limits<float>::lowest())
{
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
    if (use_recursive_growing) {
        succes &= recursiveRegionGrower();
    } else {
        success &= setupRegionGrower();
        success &= extractRegions();
    }

    clock_t end_region_grow = clock();
    double time_taken
        = double(end_region_grow - start_region_grow) / double(CLOCKS_PER_SEC);
    ROS_DEBUG_STREAM("Time taken by pointcloud RegionGrower is : "
        << std::fixed << time_taken << std::setprecision(5) << " sec "
        << std::endl);

    return success;
}

void RegionGrower::readParameters(
    march_realsense_reader::pointcloud_parametersConfig& config)
{
    number_of_neighbours
        = config.region_creator_region_growing_number_of_neighbours;
    min_cluster_size = config.region_creator_region_growing_min_cluster_size;
    max_cluster_size = config.region_creator_region_growing_max_cluster_size;
    smoothness_threshold
        = (float)config.region_creator_region_growing_smoothness_threshold;
    curvature_threshold
        = (float)config.region_creator_region_growing_curvature_threshold;
    use_recursive_growing
        = config.region_creator_region_growing_use_recursive_growing;

    debugging_ = config.debug;
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
        for (const auto& region : *region_vector_) {
            ROS_DEBUG("Total number of points in cluster %i: %lu", i,
                region.indices.size());
            i++;
        }
    }

    if (region_vector_->size() == 0) {
        ROS_WARN("Region growing algorithm found no clusters, stopping "
                 "region grower");
        return false;
    }

    return true;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RegionGrower::debug_visualisation()
{
    return region_grower.getColoredCloud();
}

bool RegionGrower::recursiveRegionGrower(
    boost::shared_ptr<RegionVector> last_region_vector, float last_tolerance)
{
    boost::shared_ptr<RegionVector> too_small_regions;
    boost::shared_ptr<RegionVector> too_large_regions;
    boost::shared_ptr<RegionVector> right_size_regions;
    segmentRegionVector(last_region_vector);

    boost::shared_ptr<RegionVector> continuing_regions;

    if (too_small->size() > 50) {
        float small_tolerance = last_tolerance / 1.1f;
        boost::shared_ptr<RegionVector> potential_region_vector
            = tryRegionGrowing(too_small, small_tolerance);
        continuing_regions.insert(continuing_regions.end(),
            potential_region_vector.begin(), potential_region_vector.end());
    }
    if (too_large->size > 50) {
        float large_tolerance = last_tolerance * 1.1f;
        RegionVector potential_region_vector
            = tryEuclideanClustering(too_large, large_tolerance);
    }

    // get the too small and large part of the region_vec
    // Add the valid ones to the region vector

    if (too_small.size > 10 || too_large.size > 10) {
        recursiveEuclideanClustering(too_small, too_large, small_tolerance)
    } else {
        // add the small and large too
    }

    // get the too small and large part of the region_vec
    // Add the valid ones to the region vector

    if (too_small.size > 10 || too_large.size > 10) {
        recursiveEuclideanClustering(too_small, too_large, large_tolerance)
    } else {
        // add the small and large too
    }
}
