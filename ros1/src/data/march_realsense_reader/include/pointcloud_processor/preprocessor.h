#ifndef MARCH_PREPROCESSOR_H
#define MARCH_PREPROCESSOR_H

#include <march_realsense_reader/pointcloud_parametersConfig.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include <tf2_ros/transform_listener.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;

class Preprocessor {
public:
    explicit Preprocessor(bool debugging);

    // This function is required to be implemented by any preprocessor
    virtual bool preprocess(PointCloud::Ptr pointcloud,
        Normals::Ptr normal_pointcloud, std::string frame_id_to_transform_to_)
        = 0;

    virtual ~Preprocessor() = default;

    // Removes a point from a pointcloud (and optionally the corresponding
    // pointcloud_normals as well) at a given index
    void removePointByIndex(int const index, const PointCloud::Ptr& pointcloud,
        const Normals::Ptr& pointcloud_normals = nullptr);

    /** This function is called upon whenever a parameter from config is
     * changed, including when launching the node
     */
    virtual void readParameters(
        march_realsense_reader::pointcloud_parametersConfig& config)
        = 0;

    PointCloud::Ptr pointcloud_;
    Normals::Ptr pointcloud_normals_;
    bool debugging_;
};

/** The SimplePreprocessor is mostly for debug purposes, it does only the most
 * vital step of transforming the pointcloud based on the /tf topic. More
 * complex preoprocessors will be made for normal usecases **/
class SimplePreprocessor : Preprocessor {
public:
    /** Basic constructor for simple preprocessor, this will also create a
    tf_listener that is required for transforming the pointcloud **/
    explicit SimplePreprocessor(bool debugging);

    // Preprocess the given pointcloud, based on parameters in the config tree
    bool preprocess(PointCloud::Ptr pointcloud, Normals::Ptr pointcloud_normals,
        std::string frame_id_to_transform_to = "foot_left") override;

protected:
    /** Calls the tf listener, to know transform at current time and transforms
     the pointcloud **/
    void transformPointCloudFromUrdf();

    // Objects needed for transformation based on URDF
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tfListener;
    std::string pointcloud_frame_id;
    std::string frame_id_to_transform_to_;
};

class NormalsPreprocessor : Preprocessor {
public:
    /** Basic constructor for normals preprocessor, this will also create a
    tf_listener that is required for transforming the pointcloud **/
    explicit NormalsPreprocessor(bool debugging);

    // Calls all subsequent methods to preprocess a pointlcoud using normal
    // vectors
    bool preprocess(PointCloud::Ptr pointcloud, Normals::Ptr pointcloud_normals,
        std::string frame_id_to_transform_to = "foot_left") override;

    void readParameters(
        march_realsense_reader::pointcloud_parametersConfig& config) override;

protected:
    // Removes points from the pointcloud such that there is only one point left
    // in a certain area (specified in the parameter file)
    bool downsample();

    // Transform the pointcloud based on the data found on the /tf topic,
    bool transformPointCloudFromUrdf(
        geometry_msgs::TransformStamped& transform_stamped);

    // Removes all points which are futher away then a certain distance from the
    // origin (specified in the parameter file)
    bool filterOnDistanceFromOrigin();

    // Estimates the normals of the pointcloud and fills the pointcloud_normals_
    // cloud with those
    bool fillNormalCloud(
        const geometry_msgs::TransformStamped& transform_stamped);

    // Removes all points which do not roughly have a normal in a certain
    // direction (specified in the parameter file)
    bool filterOnNormalOrientation();

    // Downsampling parameters
    bool voxel_grid_filter {};
    float leaf_size {};
    bool random_filter {};
    int remaining_points {};

    // Distance filter parameters
    double distance_threshold {};

    // Normal estimation parameters
    bool use_tree_search_method {};
    int number_of_neighbours {};
    double search_radius {};

    // Normal filter parameters
    double allowed_length_x {};
    double allowed_length_y {};
    double allowed_length_z {};

    // Objects needed for transformation based on URDF
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tfListener;
    std::string pointcloud_frame_id;
    std::string frame_id_to_transform_to_;
};

#endif // MARCH_PREPROCESSOR_H
