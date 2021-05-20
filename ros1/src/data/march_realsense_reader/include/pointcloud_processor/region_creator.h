#ifndef MARCH_REGION_CREATOR_H
#define MARCH_REGION_CREATOR_H

#include <march_realsense_reader/pointcloud_parametersConfig.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include <string>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;
using RegionVector = std::vector<pcl::PointIndices>;
using PointsVector = std::vector<PointCloud::Ptr>;
using NormalsVector = std::vector<Normals::Ptr>;

class RegionCreator {
public:
    explicit RegionCreator(bool debugging);
    // This function is required to be implemented by any region creator
    virtual bool createRegions(PointCloud::Ptr pointcloud,
        Normals::Ptr pointcloud_normals,
        boost::shared_ptr<PointsVector> points_vector,
        boost::shared_ptr<NormalsVector> normals_vector)
        = 0;
    virtual ~RegionCreator() = default;
    virtual pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_visualisation() = 0;

    /** This function is called upon whenever a parameter from config is
     * changed, including when launching the node
     */
    virtual void readParameters(
        march_realsense_reader::pointcloud_parametersConfig& config)
        = 0;

protected:
    PointCloud::Ptr pointcloud_;
    Normals::Ptr pointcloud_normals_;
    boost::shared_ptr<PointsVector> points_vector_;
    boost::shared_ptr<NormalsVector> normals_vector_;
    bool debugging_;
};

class RegionGrower : RegionCreator {
public:
    // Use the constructors defined in the super class
    explicit RegionGrower(bool debugging);
    /** Create cluster using the region growing algorithm, takes algorithm
     * configuration from the dynamic parameter server, and fills parameter
     * region_vector with clusters. **/
    bool createRegions(PointCloud::Ptr pointcloud,
        Normals::Ptr pointcloud_normals,
        boost::shared_ptr<PointsVector> points_vector,
        boost::shared_ptr<NormalsVector> normals_vector) override;

    /**
     * @return A pointer to a single pointcloud, with unique colours for every
     * cluster
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_visualisation() override;

    /** This function is called upon whenever a parameter from config is
     * changed, including when launching the node
     */
    void readParameters(
        march_realsense_reader::pointcloud_parametersConfig& config) override;

private:
    /**
     * Configure region growing algorithm
     */
    bool setupRegionGrower();

    /**
     * Extract clusters from region_grower object
     * @return true if successful
     */
    bool extractRegions();

    // Implements the region growing algorithm and recursively improves on too
    // small or too large regions
    bool recursiveRegionGrower(
        const boost::shared_ptr<RegionVector> last_region_vector,
        const PointCloud::Ptr last_pointcloud,
        const Normals::Ptr last_pointcloud_normals,
        const float& last_tolerance);

    // Splits a region Vector into the regions considered too large, just right,
    // and too small
    void segmentRegionVector(
        const boost::shared_ptr<RegionVector> region_vector,
        boost::shared_ptr<RegionVector> too_small_regions,
        boost::shared_ptr<RegionVector> too_large_regions,
        boost::shared_ptr<RegionVector> right_size_regions);

    // Creates a potential region vector from a pointcloud with a certain
    // tolerance
    bool getRegionVectorFromTolerance(const PointCloud::Ptr pointcloud,
        const Normals::Ptr pointcloud_normals, const float& tolerance,
        boost::shared_ptr<RegionVector> region_vector);

    // Add the right regions to the region points and region normals vectors
    void addRegionsToPointAndNormalVectors(
        const boost::shared_ptr<RegionVector> right_size_regions,
        const PointCloud::Ptr pointcloud,
        const Normals::Ptr pointcloud_normals);

    // Fill a pointcloud with the points in invalid regions
    void fillInvalidClouds(
        const boost::shared_ptr<RegionVector> invalid_region_vector,
        const PointCloud::Ptr last_pointcloud,
        const Normals::Ptr last_pointcloud_normals,
        PointCloud::Ptr invalid_pointcloud,
        Normals::Ptr invalid_pointcloud_normals);

    // Process the invalid regions with the new tolerance
    // This method makes a call to this method if the invalid region is large
    // enough
    bool processInvalidRegions(const float& next_tolerance,
        const PointCloud::Ptr invalid_pointcloud,
        Normals::Ptr invalid_pointcloud_normals,
        const boost::shared_ptr<RegionVector> invalid_regions,
        PointCloud::Ptr last_pointcloud, Normals::Ptr last_pointcloud_normals);

    // Similar to the regular setup, but the cloud, normals, and smoothness
    // threshold are not yet set.
    void setupRecursiveRegionGrower();

    // Region Growing Object
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> region_grower;

    // Region Growing configuration parameters
    int number_of_neighbours;
    int min_valid_cluster_size;
    int max_valid_cluster_size;
    int min_desired_cluster_size;
    int max_desired_cluster_size;
    float smoothness_threshold;
    float curvature_threshold;
    bool use_recursive_growing;
    float tolerance_change_factor_decrease;
    float tolerance_change_factor_increase;
    boost::shared_ptr<RegionVector> region_vector_;
};

#endif // MARCH_REGION_CREATOR_H
