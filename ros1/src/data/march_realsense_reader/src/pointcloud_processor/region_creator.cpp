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
        // First extract regions as normal to start the recursive call from the
        // first result
        success &= setupRegionGrower();
        if (!extractRegions()) {
            ROS_WARN_STREAM(
                "Extracting regions in initialization of the recursive failed. "
                "Recursive method could still work.");
        }

        region_grower = pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal>();
        success &= setupRecursiveRegionGrower();
        success &= recursiveRegionGrower(region_vector_, pointcloud_,
            pointcloud_normals_, smoothness_threshold);
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
    boost::shared_ptr<RegionVector> last_region_vector,
    PointCloud::Ptr last_pointcloud, Normals::Ptr last_pointcloud_normals,
    float last_tolerance)
{
    boost::shared_ptr<RegionVector> too_small_regions;
    boost::shared_ptr<RegionVector> too_large_regions;
    boost::shared_ptr<RegionVector> right_size_regions;
    boost::make_shared<PointCloud> too_small_pointcloud;
    boost::make_shared<Normals> too_small_pointcloud_normals;
    boost::make_shared<PointCloud> too_large_pointcloud;
    boost::make_shared<Normals> too_large_pointcloud_normals;

    // Segment the region grower into small, good, and large regions
    segmentRegionVector(last_region_vector, too_small_regions,
        too_large_regions, right_size_regions);

    // Add the right regions to the region points and region normals vectors
    addRegionsToPointAndNormalVectors(
        right_size_regions, last_pointcloud, last_pointcloud_normals);

    // Extract the invalid regions into their point clouds
    fillInvalidClouds(too_small_regions, last_pointcloud,
        last_pointcloud_normals, too_small_pointcloud,
        too_small_pointcloud_normals);
    fillInvalidClouds(too_large_regions, last_pointcloud,
        last_pointcloud_normals, too_large_pointcloud,
        too_large_pointcloud_normals);

    // Compute the new tolerances with which to do the next region growing step
    float large_tolerance = last_tolerance * 1.1f;
    float small_tolerance = last_tolerance / 1.1f;

    // Process the invalid regions with the new tolerance
    // This method makes a call to this method if the invalid region is large
    // enough
    processInvalidRegions(large_tolerance, too_small_pointcloud,
        too_small_pointcloud_normals, too_small_regions, last_pointcloud,
        last_pointcloud_normals);

    processInvalidRegions(small_tolerance, too_large_pointcloud,
        too_large_pointcloud_normals, too_large_regions, last_pointcloud,
        last_pointcloud_normals);
}

void RegionGrower::processInvalidRegions(const float& next_tolerance,
    const PointCloud::Ptr invalid_pointcloud,
    Normals::Ptr invalid_pointcloud_normals,
    const boost::shared_ptr<RegionVector> invalid_regions,
    PointCloud::Ptr last_pointcloud, Normals::Ptr last_pointcloud_normals)
{
    if (invalid_pointcloud->size() > 20) {
        // Try region growing on the invalid regions with a new tolerance
        RegionVector potential_region_vector = getRegionVectorFromTolerance(
            invalid_pointcloud, invalid_pointcloud_normals, next_tolarence);
        recursiveRegionGrower(potential_region_vector, invalid_pointcloud,
            invalid_pointcloud_normals, next_tolerance);
    } else {
        addRegionsToPointAndNormalVectors(
            invalid_regions, last_pointcloud, last_pointcloud_normals);
    }
}

// Add the right regions to the region points and region normals vectors
void RegionGrower::addRegionsToPointAndNormalVectors(
    const boost::shared_ptr<RegionVector> right_size_regions,
    const PointCloud::Ptr pointcloud, const Normals::Ptr pointcloud_normals)
{
    boost::make_shared<PointCloud> region_pointcloud;
    boost::make_shared<Normals> region_normals;

    for (pcl::PointIndices region : *right_size_regions) {
        pcl::copyPointCloud(*pointcloud, region, *region_pointcloud);
        pcl::copyPointCloud(*pointcloud_normals, region, *region_normals);

        pointcloud_vector_->push_back(region_pointcloud);
        normals_vector_->push_back(region_normals);

        region_pointcloud->clear();
        region_normals->clear();
    }
}

// Fill a pointcloud with the points in invalid regions
void RegionGrower::fillInvalidClouds(
    const boost::shared_ptr<RegionVector> invalid_region_vector,
    const PointCloud::Ptr last_pointcloud,
    const Normals::Ptr last_pointcloud_normals,
    PointCloud::Ptr invalid_pointcloud, Normals::Ptr invalid_pointcloud_normals)
{
    boost::make_shared<PointCloud> invalid_region_pointcloud;
    boost::make_shared<Normals> invalid_region_normals;

    for (pcl::PointIndices invalid_region : *invalid_region_vector) {
        pcl::copyPointCloud(
            *last_pointcloud, invalid_region, *invalid_region_pointcloud);
        pcl::copyPointCloud(
            *last_pointcloud_normals, invalid_region, *invalid_region_normals);

        *invalid_pointcloud += *invalid_region_pointcloud;
        *invalid_pointcloud_normals += *invalid_region_pointcloud_normals;

        invalid_region_pointcloud->clear();
        invalid_region_normals->clear();
    }
}

// Splits a region Vector into the regions considered too large, just right,
// and too small
void RegionGrower::segmentRegionVector(
    const boost::shared_ptr<RegionVector> region_vector,
    boost::shared_ptr<RegionVector> too_small_regions,
    boost::shared_ptr<RegionVector> too_large_regions,
    boost::shared_ptr<RegionVector> right_size_regions)
{
    for (pcl::PointIndices region : *region_vector) {
        if (region.indices.size() > 50) {
            too_large_regions.push_back(region);
        } else if (region.indices.size() < 20) {
            too_small_regions.push_back(region);
        } else {
            right_size_regions.push_back(region);
        }
    }
}

// Creates a potential region vector from a pointcloud with a certain tolerance
boost::shared_ptr<RegionVector> RegionGrower::getRegionVectorFromTolerance(
    const PointCloud::Ptr pointcloud, const Normals::Ptr pointcloud_normals,
    const float& tolerance)
{
    boost::shared_ptr<RegionVector> region_vector
        = boost::make_shared<RegionVector>();

    if (pointcloud_to_grow_on->size()
        == pointcloud_normals_to_grow_on->size()) {
        region_grower.setInputCloud(pointcloud);
        region_grower.setInputNormals(pointcloud_normals);
        region_grower.setSmoothnessThreshold(tolerance);
        region_grower.extract(*region_vector);
    } else {
        ROS_ERROR("pointcloud_to_grow_on is of size: %lu, while "
                  "pointcloud_normals_to_grow_on is "
                  "of size: %lu. Returning empty region vector.",
            pointcloud->size(), pointcloud_normals->size());
    }
    return region_vector;
}
