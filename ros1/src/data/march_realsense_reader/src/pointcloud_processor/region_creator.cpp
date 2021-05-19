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
        succes &= setupRecursiveRegionGrower()
            : succes &= recursiveRegionGrower();
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

// Similar to the regular setup, but the cloud, normals, and smoothness
// threshold are not yet set.
bool RegionGrower::setupRecursiveRegionGrower()
{
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);
    region_grower.setMinClusterSize(min_cluster_size);
    region_grower.setMaxClusterSize(max_cluster_size);
    region_grower.setSearchMethod(tree);
    region_grower.setNumberOfNeighbours(number_of_neighbours);
    region_grower.setCurvatureThreshold(curvature_threshold);
    return true;
}

// Implements the region growing algorithm and recursively improves on too small
// or too large regions
bool RegionGrower::recursiveRegionGrower(
    boost::shared_ptr<RegionVector> last_region_vector, float last_tolerance)
{
    boost::shared_ptr<RegionVector> too_small_regions;
    boost::shared_ptr<RegionVector> too_large_regions;
    boost::shared_ptr<RegionVector> right_size_regions;
    segmentRegionVector(last_region_vector, too_small_regions,
        too_large_regions, right_size_regions);

    region_vector_.insert(region_vector_.end(), right_size_regions.begin(),
        right_size_regions.end());

    total_size_of_small_regions = getTotalRegionVectorSize(too_small_regions);
    total_size_of_large_regions = getTotalRegionVectorSize(too_large_regions);

    if (total_size_of_small_regions > 50) {
        float large_tolerance = last_tolerance * 1.1f;
        // Try region growing on the small regions with a larger tolerance
        boost::shared_ptr<RegionVector> potential_region_vector
            = doRecursiveRegionGrowingStep(too_small_regions, large_tolerance);
        recursiveRegionGrower(potential_region_vector, large_tolerance);
    } else {
        region_vector_.insert(region_vector_.end(), too_small_regions.begin(),
            too_small_regions.end());
    }

    if (total_size_of_small_regions > 50) {
        float small_tolerance = last_tolerance / 1.1f;
        // Try region growing on the large regions with a smaller tolerance
        RegionVector potential_region_vector
            = doRecursiveRegionGrowingStep(large_region, small_tolerance);
        recursiveRegionGrower(potential_region_vector, small_tolerance);
    } else {
        region_vector_.insert(region_vector_.end(), too_large_regions.begin(),
            too_large_regions.end());
    }
}

// Find the total number of points in a region vector
int RegionGrower::getTotalRegionVectorSize(
    boost::shared_ptr<RegionVector> region_vector)
{
    int total = 0;
    for (pcl::PointIndices region : *region_vector) {
        total += region.indices.size();
    }
    return total;
}

void RegionGrower::segmentRegionVector(
    boost::shared_ptr<RegionVector> region_vector)
{
}

boost::shared_ptr<RegionVector> RegionGrower::doRecursiveRegionGrowingStep(
    pcl::PointIndices region, float tolerance)
{
    boost::shared_ptr<RegionVector> region_vector
        = boost::make_shared<RegionVector>();
    PointCloud::Ptr pointcloud_to_grow_on = boost::make_shared<PointCloud>();
    PointCloud::Ptr pointcloud_normals_to_grow_on
        = boost::make_shared<Normals>();

    pcl::copyPointCloud(*region_to_grow_on, region, *region_points_);
    pcl::copyPointCloud(*region_normals_to_grow_on, region, *region_normals_);

    if (pointcloud_to_grow_on->size()
        == pointcloud_normals_to_grow_on->size()) {
        region_grower.setInputCloud(pointcloud_to_grow_on);
        region_grower.setInputNormals(pointcloud_normals_to_grow_on);
        region_grower.setSmoothnessThreshold(tolerance);
        region_grower.extract(*region_vector);
    } else {
        ROS_ERROR("pointcloud_to_grow_on is of size: %lu, while "
                  "pointcloud_normals_to_grow_on is "
                  "of size: %lu. Returning empty region vector.",
            pointcloud_to_grow_on->size(),
            pointcloud_normals_to_grow_on->size());
    }
    return region_vector;
}
