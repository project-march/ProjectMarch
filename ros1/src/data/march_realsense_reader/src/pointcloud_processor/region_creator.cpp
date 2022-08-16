#include "pointcloud_processor/region_creator.h"
#include <ctime>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <ros/ros.h>
#include <utilities/output_utilities.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;
using RegionVector = std::vector<pcl::PointIndices>;

// Construct a basic RegionCreator class
RegionCreator::RegionCreator(bool debugging)
    : debugging_ { debugging }
{
}

// Construct a basic RegionGrower class
RegionGrower::RegionGrower(bool debugging)
    : RegionCreator(debugging)
    , number_of_neighbours(-1)
    , number_of_neighbours_no_seeds(-1)
    , min_valid_cluster_size(-1)
    , max_valid_cluster_size(-1)
    , min_desired_cluster_size(-1)
    , max_desired_cluster_size(-1)
    , smoothness_threshold(std::numeric_limits<float>::lowest())
    , curvature_threshold(std::numeric_limits<float>::lowest())
    , smoothness_threshold_lower_bound(-1)
    , smoothness_threshold_upper_bound(-1)
    , use_recursive_growing(false)
    , tolerance_change_factor_decrease(0)
    , tolerance_change_factor_increase(0)
    , number_of_recursive_calls(0)
    , use_no_seed_growing(true)
{
}

bool RegionGrower::createRegions(const PointCloud::Ptr pointcloud, const Normals::Ptr pointcloud_normals,
    const RealSenseCategory realsense_category, boost::shared_ptr<PointsVector> points_vector,
    boost::shared_ptr<NormalsVector> normals_vector)
{
    pointcloud_ = pointcloud;
    pointcloud_normals_ = pointcloud_normals;
    realsense_category_.emplace(realsense_category);
    points_vector_ = points_vector;
    normals_vector_ = normals_vector;
    ROS_DEBUG_STREAM("Creating regions with region growing");

    clock_t start_region_grow = clock();

    bool success = true;
    // First extract regions as normal to potentially start the recursive call
    // from the first result to improve it
    success &= setupRegionGrower();
    success &= extractRegions();
    if (use_recursive_growing && success) {
        // reinitialize the region grower to start with an empty object at the
        // start of the recursive call
        region_grower = pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal>();
        setupRecursiveRegionGrower();
        success &= recursiveRegionGrower(region_vector_, pointcloud_, pointcloud_normals_, smoothness_threshold);
    }

    clock_t end_region_grow = clock();

    if (debugging_) {
        ROS_DEBUG("Total number of clusters found: %lu", points_vector_->size());
        std::vector<int> length_vector(points_vector_->size());
        int right_size_regions_count = 0;
        int too_small_regions_count = 0;
        int too_large_regions_count = 0;
        for (const PointCloud::Ptr& region : *points_vector_) {
            if (region->size() > 0) {
                length_vector.push_back(region->size());
                if (region->size() > max_desired_cluster_size) {
                    ++too_large_regions_count;
                } else if (region->size() < min_desired_cluster_size) {
                    ++too_small_regions_count;
                } else {
                    ++right_size_regions_count;
                }
            }
        }
        ROS_DEBUG_STREAM("All the lengths of the regions are in the following vector: "
            << output_utilities::vectorToString(length_vector));

        if (use_recursive_growing) {
            ROS_DEBUG_STREAM("The number of recursive calls made is: " << number_of_recursive_calls);
            ROS_DEBUG_STREAM("There were " << right_size_regions_count << " regions with the right size, "
                                           << too_small_regions_count << " to small regions and "
                                           << too_large_regions_count << " too large regions");
        }

        double time_taken = double(end_region_grow - start_region_grow) / double(CLOCKS_PER_SEC);
        ROS_DEBUG_STREAM("Time taken by pointcloud RegionGrower is : " << std::fixed << time_taken
                                                                       << std::setprecision(5) << " sec " << std::endl);
    }

    return success;
}

void RegionGrower::readParameters(march_realsense_reader::pointcloud_parametersConfig& config)
{
    number_of_neighbours = config.region_creator_region_growing_number_of_neighbours;
    number_of_neighbours_no_seeds = config.region_creator_region_growing_number_of_neighbours_no_seeds;
    min_valid_cluster_size = config.region_creator_region_growing_min_valid_cluster_size;
    max_valid_cluster_size = config.region_creator_region_growing_max_valid_cluster_size;
    min_desired_cluster_size = config.region_creator_region_growing_min_desired_cluster_size;
    max_desired_cluster_size = config.region_creator_region_growing_max_desired_cluster_size;
    smoothness_threshold = (float)config.region_creator_region_growing_smoothness_threshold;
    smoothness_threshold_lower_bound = (float)config.region_creator_region_growing_smoothness_threshold_lower_bound;
    smoothness_threshold_upper_bound = (float)config.region_creator_region_growing_smoothness_threshold_upper_bound;
    curvature_threshold = (float)config.region_creator_region_growing_curvature_threshold;
    use_recursive_growing = config.region_creator_region_growing_use_recursive_growing;
    tolerance_change_factor_increase = (float)config.region_creator_region_growing_tolerance_change_factor_increase;
    tolerance_change_factor_decrease = (float)config.region_creator_region_growing_tolerance_change_factor_decrease;
    use_no_seed_growing = config.region_creator_region_growing_use_no_seed_growing;

    debugging_ = config.debug;
}

bool RegionGrower::setupRegionGrower()
{
    if (pointcloud_->size() == pointcloud_normals_->size()) {
        pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        region_grower.setSearchMethod(tree);
        region_grower.setInputCloud(pointcloud_);
        region_grower.setInputNormals(pointcloud_normals_);
        region_grower.setSmoothnessThreshold(smoothness_threshold);
        if ((realsense_category_.value() == RealSenseCategory::ramp_up
                || realsense_category_.value() == RealSenseCategory::ramp_down)
            && use_no_seed_growing) {
            region_grower.setCurvatureThreshold(/*curvature=*/-1);
            region_grower.setNumberOfNeighbours(number_of_neighbours_no_seeds);
        } else {
            region_grower.setNumberOfNeighbours(number_of_neighbours);
            region_grower.setCurvatureThreshold(curvature_threshold);
        }
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
    region_vector_ = std::make_unique<RegionVector>();
    region_grower.extract(*region_vector_);

    if (region_vector_->size() == 0) {
        ROS_WARN("Region growing algorithm found no clusters, stopping "
                 "region grower");
        return false;
    }

    if (!use_recursive_growing) {
        points_vector_->reserve(region_vector_->size());
        normals_vector_->reserve(region_vector_->size());

        addRegionsToPointAndNormalVectors(region_vector_, pointcloud_, pointcloud_normals_);
    }

    return true;
}

ColoredCloud::Ptr RegionGrower::debug_visualisation()
{
    if (!use_recursive_growing) {
        return region_grower.getColoredCloud();
    } else {
        ColoredCloud::Ptr colored_cloud = boost::make_shared<ColoredCloud>();
        ColoredCloud::Ptr colored_region = boost::make_shared<ColoredCloud>();
        for (PointCloud::Ptr& region_point : *points_vector_) {
            // Color the hull with a random color (r, g and b in [0, 255]))
            int number_of_colors = 255;
            // clang-tidy linter cert-msc30-c and cert-msc50-cpp say that rand()
            // is not a uniform distribution. This is not something that is
            // important here, therefore these lines can ignore this linter
            // rule.
            int r = (rand() % number_of_colors); // NOLINT
            int g = (rand() % number_of_colors); // NOLINT
            int b = (rand() % number_of_colors); // NOLINT
            colored_region->resize(region_point->size());
            for (pcl::PointXYZ point : *region_point) {
                pcl::PointXYZRGB colored_point;
                colored_point.x = point.x;
                colored_point.y = point.y;
                colored_point.z = point.z;
                colored_point.r = r;
                colored_point.g = g;
                colored_point.b = b;
                colored_region->push_back(colored_point);
            }
            *colored_cloud += *colored_region;
        }
        return colored_cloud;
    }
}

// Similar to the regular setup, but the cloud, normals, and smoothness
// threshold are not yet set.
void RegionGrower::setupRecursiveRegionGrower()
{
    number_of_recursive_calls = 0;

    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    region_grower.setSearchMethod(tree);
    // Set the number of neighbours smaller then teh min valid cluster size to
    // avoid combining small regions which are far apart
    region_grower.setNumberOfNeighbours(min_valid_cluster_size - 1);
    region_grower.setCurvatureThreshold(curvature_threshold);
}

// Implements the region growing algorithm and recursively improves on too small
// or too large regions
bool RegionGrower::recursiveRegionGrower(const std::unique_ptr<RegionVector>& last_region_vector,
    const PointCloud::Ptr& last_pointcloud, const Normals::Ptr& last_pointcloud_normals, const float& last_tolerance)
{
    number_of_recursive_calls++;

    bool success = true;
    std::unique_ptr<RegionVector> too_small_regions = std::make_unique<RegionVector>();
    std::unique_ptr<RegionVector> too_large_regions = std::make_unique<RegionVector>();
    std::unique_ptr<RegionVector> right_size_regions = std::make_unique<RegionVector>();
    PointCloud::Ptr too_small_pointcloud = boost::make_shared<PointCloud>();
    Normals::Ptr too_small_pointcloud_normals = boost::make_shared<Normals>();
    PointCloud::Ptr too_large_pointcloud = boost::make_shared<PointCloud>();
    Normals::Ptr too_large_pointcloud_normals = boost::make_shared<Normals>();

    // Segment the region grower into small, good, and large regions
    segmentRegionVector(last_region_vector, too_small_regions, too_large_regions, right_size_regions);

    // Add the right regions to the region points and region normals vectors
    addRegionsToPointAndNormalVectors(right_size_regions, last_pointcloud, last_pointcloud_normals);

    // Extract the invalid regions into their point clouds
    fillInvalidClouds(too_small_regions, last_pointcloud, last_pointcloud_normals, too_small_pointcloud,
        too_small_pointcloud_normals);
    fillInvalidClouds(too_large_regions, last_pointcloud, last_pointcloud_normals, too_large_pointcloud,
        too_large_pointcloud_normals);

    // Compute the new tolerances with which to do the next region growing step
    float large_tolerance = last_tolerance * tolerance_change_factor_increase;
    float small_tolerance = last_tolerance * tolerance_change_factor_decrease;

    // Process the invalid regions with the new tolerance
    // The processInvalidRegions method makes a call to the
    // recursiveRegionGrower method if the invalid region is large enough and
    // the tolerance is within limits
    success &= processInvalidRegions(large_tolerance, too_small_pointcloud, too_small_pointcloud_normals,
        too_small_regions, last_pointcloud, last_pointcloud_normals);

    success &= processInvalidRegions(small_tolerance, too_large_pointcloud, too_large_pointcloud_normals,
        too_large_regions, last_pointcloud, last_pointcloud_normals);

    return success;
}

bool RegionGrower::processInvalidRegions(const float& next_tolerance, const PointCloud::Ptr& invalid_pointcloud,
    const Normals::Ptr& invalid_pointcloud_normals, const std::unique_ptr<RegionVector>& invalid_regions,
    const PointCloud::Ptr& last_pointcloud, const Normals::Ptr& last_pointcloud_normals)
{
    // If the invalid pointcloud size is large enough and the next tolerance is
    // reasonable, grow new regions on the invalid pointcloud
    if (invalid_pointcloud->size() > min_desired_cluster_size && next_tolerance < smoothness_threshold_upper_bound
        && next_tolerance < smoothness_threshold_upper_bound) {
        // Try region growing on the invalid regions with a new tolerance
        std::unique_ptr<RegionVector> potential_region_vector = std::make_unique<RegionVector>();
        bool success = getRegionVectorFromTolerance(
            invalid_pointcloud, invalid_pointcloud_normals, next_tolerance, potential_region_vector);
        if (success) {
            // Investigate if the newly found region vector has valid
            // regions and process its invalid regions again
            success &= recursiveRegionGrower(
                potential_region_vector, invalid_pointcloud, invalid_pointcloud_normals, next_tolerance);
        }
        return success;
    } else {
        addRegionsToPointAndNormalVectors(invalid_regions, last_pointcloud, last_pointcloud_normals);
        return true;
    }
}

// Add the right regions to the region points and region normals vectors
void RegionGrower::addRegionsToPointAndNormalVectors(const std::unique_ptr<RegionVector>& right_size_regions,
    const PointCloud::Ptr& pointcloud, const Normals::Ptr& pointcloud_normals)
{

    for (pcl::PointIndices& region : *right_size_regions) {
        if (region.indices.size() > min_valid_cluster_size && region.indices.size() < max_valid_cluster_size) {
            PointCloud::Ptr region_pointcloud = boost::make_shared<PointCloud>();
            Normals::Ptr region_normals = boost::make_shared<Normals>();
            pcl::copyPointCloud(*pointcloud, region, *region_pointcloud);
            pcl::copyPointCloud(*pointcloud_normals, region, *region_normals);

            points_vector_->push_back(region_pointcloud);
            normals_vector_->push_back(region_normals);
        }
    }
}

// Fill a pointcloud with all the points in invalid regions
void RegionGrower::fillInvalidClouds(const std::unique_ptr<RegionVector>& invalid_region_vector,
    const PointCloud::Ptr& last_pointcloud, const Normals::Ptr& last_pointcloud_normals,
    PointCloud::Ptr& invalid_pointcloud, Normals::Ptr& invalid_pointcloud_normals)
{
    // Initialize the clouds corresponding to the regions
    PointCloud::Ptr invalid_region_pointcloud = boost::make_shared<PointCloud>();
    Normals::Ptr invalid_region_normals = boost::make_shared<Normals>();

    for (pcl::PointIndices& invalid_region : *invalid_region_vector) {
        // Extract the points and normals of the last cloud which were
        // invalid into the region clouds
        pcl::copyPointCloud(*last_pointcloud, invalid_region, *invalid_region_pointcloud);
        pcl::copyPointCloud(*last_pointcloud_normals, invalid_region, *invalid_region_normals);

        // Add the region clouds together in a cloud containing all invalid
        // points on which region growing can be executed again
        *invalid_pointcloud += *invalid_region_pointcloud;
        *invalid_pointcloud_normals += *invalid_region_normals;

        // Clear the region clouds so they can be used again
        invalid_region_pointcloud->clear();
        invalid_region_normals->clear();
    }
}

// Splits a region Vector into the regions considered too large, just right,
// and too small
void RegionGrower::segmentRegionVector(const std::unique_ptr<RegionVector>& region_vector,
    std::unique_ptr<RegionVector>& too_small_regions, std::unique_ptr<RegionVector>& too_large_regions,
    std::unique_ptr<RegionVector>& right_size_regions)
{
    for (pcl::PointIndices& region : *region_vector) {
        if (region.indices.size() > max_desired_cluster_size) {
            too_large_regions->push_back(region);
        } else if (region.indices.size() < min_desired_cluster_size) {
            too_small_regions->push_back(region);
        } else {
            right_size_regions->push_back(region);
        }
    }
}

// Creates a potential region vector from a pointcloud with a certain
// tolerance
bool RegionGrower::getRegionVectorFromTolerance(const PointCloud::Ptr& pointcloud,
    const Normals::Ptr& pointcloud_normals, const float& tolerance, std::unique_ptr<RegionVector>& region_vector)
{
    if (pointcloud->size() == pointcloud_normals->size()) {
        region_grower.setInputCloud(pointcloud);
        region_grower.setInputNormals(pointcloud_normals);
        region_grower.setSmoothnessThreshold(tolerance);
        region_grower.extract(*region_vector);
    } else {
        ROS_ERROR("pointcloud_to_grow_on is of size: %lu, while "
                  "pointcloud_normals_to_grow_on is "
                  "of size: %lu. Returning empty region vector.",
            pointcloud->size(), pointcloud_normals->size());
        return false;
    }
    return true;
}
