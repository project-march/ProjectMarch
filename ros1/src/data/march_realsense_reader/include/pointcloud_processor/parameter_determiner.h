#ifndef MARCH_PARAMETER_DETERMINER_H
#define MARCH_PARAMETER_DETERMINER_H
#include "march_shared_msgs/GetGaitParameters.h"
#include "utilities/realsense_category_utilities.h"
#include "utilities/transform_utilities.h"
#include <march_realsense_reader/pointcloud_parametersConfig.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include <string>
#include <visualization_msgs/MarkerArray.h>

using PointNormalCloud = pcl::PointCloud<pcl::PointNormal>;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;
using Region = pcl::PointIndices;
using PlaneCoefficients = pcl::ModelCoefficients;
using Hull = pcl::PointCloud<pcl::PointXYZ>;
using Polygon = std::vector<pcl::Vertices>;
using RegionVector = std::vector<Region>;
using PlaneCoefficientsVector = std::vector<PlaneCoefficients::Ptr>;
using HullVector = std::vector<Hull::Ptr>;
using PolygonVector = std::vector<Polygon>;
using GaitParameters = march_shared_msgs::GaitParameters;

class ParameterDeterminer {
public:
    explicit ParameterDeterminer(bool debugging);
    /** This function is required to be implemented by any plane finder **/
    virtual bool determineParameters(boost::shared_ptr<PlaneCoefficientsVector> const plane_coefficients_vector,
        boost::shared_ptr<HullVector> const hull_vector, boost::shared_ptr<PolygonVector> const polygon_vector,
        RealSenseCategory const realsense_category, boost::shared_ptr<GaitParameters> gait_parameters,
        std::string frame_id_to_transform_to, std::string subgait_name)
        = 0;

    virtual ~ParameterDeterminer() = default;

    /** This function is called upon whenever a parameter from config is
     * changed, including when launching the node
     */
    virtual void readParameters(march_realsense_reader::pointcloud_parametersConfig& config) = 0;

    visualization_msgs::MarkerArray debug_marker_array;

protected:
    boost::shared_ptr<PlaneCoefficientsVector> plane_coefficients_vector_;
    boost::shared_ptr<HullVector> hull_vector_;
    boost::shared_ptr<PolygonVector> polygon_vector_;
    std::optional<RealSenseCategory> realsense_category_ = std::nullopt;
    boost::shared_ptr<GaitParameters> gait_parameters_;
    bool debugging_;
    std::string frame_id_to_transform_to_;
};

/** The hull parameter determiner
 *
 */
class HullParameterDeterminer : public ParameterDeterminer {
public:
    /** Basic constructor for ParameterDeterminer preprocessor **/
    explicit HullParameterDeterminer(bool debugging);

    /** This function should take in a pointcloud with matching normals and
     * hulls, and turn this into a location where the foot can be placed,
     * from this location, gaits parameters should be made. **/
    bool determineParameters(boost::shared_ptr<PlaneCoefficientsVector> const plane_coefficients_vector,
        boost::shared_ptr<HullVector> const hull_vector, boost::shared_ptr<PolygonVector> const polygon_vector,
        RealSenseCategory const realsense_category, boost::shared_ptr<GaitParameters> gait_parameters,
        std::string frame_id_to_transform_to, std::string subgait_name) override;

    /** This function is called upon whenever a parameter from config is
     * changed, including when launching the node
     */
    void readParameters(march_realsense_reader::pointcloud_parametersConfig& config) override;

    /** Takes a point cloud, with an expected z = 0, of potential foot locations
     * and returns the valid foot locations with associated height and normal
     * vector. Result indicates whether every original point ends up being
     * valid.**/
    bool cropCloudToHullVector(PointCloud::Ptr const& input_cloud, const PointNormalCloud::Ptr& output_cloud);

protected:
    // Get relevant information from the environment for the current category
    // (e.g. sit -> get sit height, stair -> get foot location)
    bool getObstacleInformation();

    // Get the optimal foot location by finding which possible foot location is
    // closest to the most desirable foot location
    bool getOptimalFootLocation();

    // Get the optimal foot location by selecting one from the possible
    // locations
    bool getOptimalFootLocationFromPossibleLocations();

    // From the possible foot locations, find which one is closes to the most
    // desirable location
    bool getPossibleMostDesirableLocation();

    // Compute the optimal foot location as if one were not limited by anything.
    bool getGeneralMostDesirableLocation();

    // Create a point cloud with points on the ground where the points represent
    // where it should be checked if there is a valid foot location
    bool getOptionalFootLocations(const PointCloud::Ptr& cloud_to_fill);

    // Crops a single point to a hull vector.
    bool cropPointToHullVector(pcl::PointXYZ const input_point, const PointNormalCloud::Ptr& output_cloud);

    // Crops a cloud to a hull vector, but only puts each input point in
    // the highest hull it falls into
    bool cropCloudToHullVectorUnique(PointCloud::Ptr const& input_cloud, const PointNormalCloud::Ptr& output_cloud);

    // Elevate the points so they have z coordinate as if they lie on the
    // plane of the hull
    bool addZCoordinateToCloudFromPlaneCoefficients(const PointCloud::Ptr& input_cloud,
        const PlaneCoefficients::Ptr& plane_coefficients, const PointCloud::Ptr& elevated_cloud);

    // Remove all points from a cloud which do not fall in the hull
    bool cropCloudToHull(const PointCloud::Ptr& elevated_cloud, const Hull::Ptr& hull, const Polygon& polygon);

    // Add normals to the elevated cloud which correspond to the normal vector
    // of the plane
    bool addNormalToCloudFromPlaneCoefficients(const PointCloud::Ptr& elevated_cloud,
        const PlaneCoefficients::Ptr& plane_coefficients, const PointNormalCloud::Ptr& elevated_cloud_with_normals);

    // Find the parameters from the foot location by finding at what percentage
    // of the end points it is
    bool getGaitParametersFromLocation();

    // Verify if there is support for the entire foot around the possible foot
    // location
    bool entireFootCanBePlaced(pcl::PointNormal possible_foot_location);

    // Fill a point cloud with vertices of the foot on the ground around a
    // possible foot location
    void fillFootPointCloud(const PointCloud::Ptr& foot_pointcloud, pcl::PointNormal possible_foot_location);

    // Verify that a possible foot location is valid for the requested gait
    bool isValidLocation(pcl::PointNormal possible_foot_location);

    // get the distance from a location to some object depending on the obstacle
    bool getDistanceToObject(pcl::PointNormal possible_foot_location, double& distance);

    // Find the stairs up parameters from the foot locations
    bool getGaitParametersFromFootLocationStairs();

    // Find the ramp parameter from the foot locations
    bool getGaitParametersFromRampSlope();

    // Find the sit parameter from the sit height
    bool getGaitParametersFromSitHeight();

    // Find the curb parameter from the curb height
    bool getGaitParametersFromCurbHeight();

    // Fill the foot locations to try cloud with a line of points from (start,
    // 0) to (end, 0)
    bool fillOptionalFootLocationCloud(const PointCloud::Ptr& cloud_to_fill, float start, float end);

    // Set the gait dimension variables to the relevant value
    void initializeGaitDimensions();

    // Initialize the debug output marker lists to easily add them during
    // computations
    void initializeDebugOutput();

    // Initialize a single marker list with a certain id
    visualization_msgs::Marker initializeMarkerListWithId(int id);

    // Add the gait information to the marker array
    void addDebugGaitInformation();

    // Add the marker lists to the marker array
    void addDebugMarkersToArray();

    // Get the slope of a ramp based on the orientation of points on the ramp
    bool getRampSlope();

    // Calculate the slope of a ramp using the normals of the
    // possible_foot_locations cloud
    bool calculateRampSlope();

    // Computes the average normal of a given input cloud
    bool getAverageNormal(const PointNormalCloud::Ptr& possible_foot_locations, pcl::Normal& average_normal);

    // Computes the slope in the x direction in degrees from a normal vector
    bool getSlopeFromNormals(const pcl::Normal& normal, float& slope);

    // The sit analogue of getOptimalFootLocation, find the height at which to
    // sit
    bool getSitHeight();

    // The curb analogue of getOptimalFootLocation, find the height of the curb
    bool getCurbHeight();

    // Fill a cloud with a grid of points where to look for exo support
    bool fillSitGrid(PointCloud::Ptr& sit_grid);

    // Get the median height value of a point cloud
    bool getMedianHeightCloud(const PointNormalCloud::Ptr& cloud, float& median_height);

    // Transform valid gait information into a parameter
    float calculateParameter(const float& valid_value, const float& minimum_value, const float& maximum_val);

    // Trim exo support cloud to only contain reachable points
    void getValidExoSupport(
        const PointNormalCloud::Ptr& potential_exo_support_points, PointNormalCloud::Ptr& exo_support_points);

    // Check which points on the curb are reachable and give foot support
    void getValidPointsOnCurb(const PointNormalCloud::Ptr& points_on_curb, PointNormalCloud::Ptr& valid_points_on_curb);

    // Updates the gait information limits after calling a transform to the
    // fixed frame
    bool transformGaitInformation();

    // All relevant parameters
    int hull_dimension {};
    int number_of_optional_foot_locations {};
    float min_x_stairs_up {};
    float max_x_stairs_up {};
    float min_z_stairs_up {};
    float max_z_stairs_up {};
    float min_x_stairs {};
    float max_x_stairs {};
    float min_z_stairs {};
    float max_z_stairs {};
    float allowed_deviation_from_reachable_stair {};
    float min_x_stairs_world {};
    float max_x_stairs_world {};
    float min_z_stairs_world {};
    float max_z_stairs_world {};
    float y_deviation_right_foot {};
    float y_deviation_foot {};
    float foot_length_back {};
    float foot_length_front {};
    float foot_width {};
    float max_allowed_z_deviation_foot {};
    float max_ramp_search {};
    float min_ramp_search {};
    float max_slope {};
    float min_slope {};
    float allowed_deviation_from_reachable_ramp {};
    float min_sit_height {};
    float max_sit_height {};
    float allowed_deviation_from_reachable_bench {};
    float min_sit_height_world {};
    float max_sit_height_world {};
    float min_x_search_sit {};
    float max_x_search_sit {};
    float search_y_deviation_sit {};
    float sit_pos_x {};
    float sit_pos_y {};
    float sit_grid_size {};
    float minimal_needed_support_sit {};
    bool general_most_desirable_location_is_mid {};
    bool general_most_desirable_location_is_small {};
    float sit_height {};
    float max_curb_up_height {};
    float max_curb_height {};
    float max_curb_height_world {};
    float min_curb_up_height {};
    float min_curb_height {};
    float min_curb_height_world {};
    float allowed_deviation_from_reachable_curb {};
    float curb_position_x {};
    float curb_position_y {};
    float min_curb_search {};
    float max_curb_search {};
    float curb_height {};
    float ramp_slope {};

    std::string subgait_name_;

    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tfListener;
    std::unique_ptr<Transformer> transformer_;

    visualization_msgs::Marker foot_locations_to_try_marker_list;
    visualization_msgs::Marker possible_foot_locations_marker_list;
    visualization_msgs::Marker gait_information_marker_list;
    visualization_msgs::Marker optimal_location_marker;
    std::shared_ptr<pcl::PointXYZ> most_desirable_foot_location_;

    pcl::PointNormal optimal_foot_location;
    PointNormalCloud::Ptr possible_foot_locations;
    PointCloud::Ptr sit_grid;
    PointCloud::Ptr foot_locations_to_try;
    PointCloud::Ptr gait_information_cloud;
    PointNormalCloud::Ptr points_on_ramp;
    PointNormalCloud::Ptr points_on_curb;
    PointCloud::Ptr locations_to_compute_ramp;
};

/** The simple parameter determiner
 *
 */
class SimpleParameterDeterminer : ParameterDeterminer {
public:
    /** Use the constructors defined in the super class **/
    using ParameterDeterminer::ParameterDeterminer;
    /** A Simple implementation which return parameters of 0.5 **/
    bool determineParameters(boost::shared_ptr<PlaneCoefficientsVector> const plane_coefficients_vector,
        boost::shared_ptr<HullVector> const hull_vector, boost::shared_ptr<PolygonVector> const polygon_vector,
        RealSenseCategory const realsense_category, boost::shared_ptr<GaitParameters> gait_parameters,
        std::string frame_id_to_transform_to, std::string subgait_name) override;
};

#endif // MARCH_PARAMETER_DETERMINER_H
