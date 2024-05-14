/*
 * P.Fankhauser, M.Bloesch, and M.Hutter, 
 * "Probabilistic Terrain Mapping for Mobile Robots with Uncertain Localization",
 * in IEEE Robotics and Automation Letters (RA-L)
 *
 * MARCH functionality and ROS2 porting by Alexander Andonov
 */
#pragma once

// ROS
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Eigen
#include <Eigen/Core>

// Kindr
#include <kindr/Core>

// STL
#include <memory>
#include <string>
#include <unordered_map>

// PCL
#include "march_vision/elevation_mapping/point_XYZ_RGB_confidence_ratio.hpp"
#include "march_vision/elevation_mapping/thread_safe_data_wrapper.hpp"

namespace elevation_mapping {

/*!
 * Generic Sensor processor base class. Provides functionalities
 * common to all sensors and defines the interface for specialized
 * sensor processor classes.
 * Cleans the point cloud, transforms it to a desired frame, and
 * computes the measurement variances based on a sensor model in
 * the desired frame.
 */
class SensorProcessorBase {
 public:
  using Ptr = std::unique_ptr<SensorProcessorBase>;
  friend class ElevationMapping;
  friend class Input;

  struct GeneralParameters {
    std::string robotBaseFrameId_;
    std::string mapFrameId_;

    GeneralParameters(std::string robotBaseFrameId = "robot", std::string mapFrameId = "map")
        : robotBaseFrameId_(std::move(robotBaseFrameId)), mapFrameId_(std::move(mapFrameId)) {}
  };

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   * @param generalConfig General parameters that the sensor processor must know in order to work. // TODO (magnus) improve documentation.
   * @param inputSourceName input source name
   */
  SensorProcessorBase(std::shared_ptr<rclcpp::Node>& nodeHandle, const GeneralParameters& generalConfig);

  /*!
   * Destructor.
   */
  virtual ~SensorProcessorBase();

  /*!
   * Processes the point cloud.
   * @param[in] pointCloudInput the input point cloud.
   * @param[in] robotPoseCovariance covariance matrix of robot position
   * @param[out] pointCloudMapFrame the processed point cloud.
   * @param[out] variances the measurement variances expressed in the target frame.
   * @param[in] sensorFrame the frame id of the sensor.
   * @return true if successful.
   */
  bool process(const PointCloudType::ConstPtr pointCloudInput, const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
               const PointCloudType::Ptr pointCloudMapFrame, Eigen::VectorXf& variances, std::string sensorFrame);

  /*!
   * Checks if a valid tf transformation was received since startup.
   * @return True if there was one valid tf transformation.
   */
  bool isTfAvailableInBuffer() { return firstTfAvailable_; }

 protected:
  /*!
   * Reads and verifies the parameters.
   * @param input source name
   * @return true if successful.
   */
  virtual bool readParameters(std::string& inputSourceName);

  /*!
   * Filters the point cloud regardless of the sensor type. Removes NaN values.
   * Optionally, applies voxelGridFilter to reduce number of points in
   * the point cloud.
   * @param pointCloud the point cloud to clean.
   * @return true if successful.
   */
  bool filterPointCloud(const PointCloudType::Ptr pointCloud);

  /*!
   * Sensor specific point cloud cleaning.
   * @param pointCloud the point cloud to clean.
   * @return true if successful.
   */
  virtual bool filterPointCloudSensorType(const PointCloudType::Ptr pointCloud);

  /*!
   * Computes the elevation map height variances for each point in a point cloud with the
   * sensor model and the robot pose covariance.
   * @param[in] pointCloud the point cloud for which the variances are computed.
   * @param[in] robotPoseCovariance the robot pose covariance matrix.
   * @param[out] variances the elevation map height variances.
   * @return true if successful.
   */
  virtual bool computeVariances(const PointCloudType::ConstPtr pointCloud, const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
                                Eigen::VectorXf& variances) = 0;

  /*!
   * Update the transformations for a given time stamp.
   * @param timeStamp the time stamp for the transformation.
   * @return true if successful.
   */
  bool updateTransformations(const rclcpp::Time& timeStamp);

  /*!
   * Transforms the point cloud the a target frame.
   * @param[in] pointCloud the point cloud to be transformed.
   * @param[out] pointCloudTransformed the resulting point cloud after transformation.
   * @param[in] targetFrame the desired target frame.
   * @return true if successful.
   */
  bool transformPointCloud(PointCloudType::ConstPtr pointCloud, PointCloudType::Ptr pointCloudTransformed, const std::string& targetFrame);

  /*!
   * Removes points with z-coordinate above a limit in map frame.
   * @param[in/out] pointCloud the point cloud to be cropped.
   */
  void removePointsOutsideLimits(PointCloudType::ConstPtr reference, std::vector<PointCloudType::Ptr>& pointClouds);

  //! ROS nodehandle.
  std::shared_ptr<rclcpp::Node>& nodeHandle_;

  //! TF transform listener and buffer.
  std::shared_ptr<tf2_ros::Buffer> transformBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> transformListener_;

  //! Rotation from Base to Sensor frame (C_SB)
  kindr::RotationMatrixD rotationBaseToSensor_;

  //! Translation from Base to Sensor in Base frame (B_r_BS)
  kindr::Position3D translationBaseToSensorInBaseFrame_;

  //! Rotation from (elevation) Map to Base frame (C_BM)
  kindr::RotationMatrixD rotationMapToBase_;

  //! Translation from Map to Base in Map frame (M_r_MB)
  kindr::Position3D translationMapToBaseInMapFrame_;

  //! Transformation from Sensor to Map frame
  Eigen::Affine3d transformationSensorToMap_;

  GeneralParameters generalParameters_;

  //! TF frame id of the range sensor for the point clouds.
  std::string sensorFrameId_;

  /*//! Ignore points above this height in map frame.
  double ignorePointsUpperThreshold_;

  //! Ignore points below this height in map frame.
  double ignorePointsLowerThreshold_;*/

  struct Parameters {
    //! Ignore points above this height in map frame.
    double ignorePointsUpperThreshold_{std::numeric_limits<double>::infinity()};

    //! Ignore points below this height in map frame.
    double ignorePointsLowerThreshold_{-std::numeric_limits<double>::infinity()};

    //! Ignore points inside this box with min_x in map frame.
    double ignorePointsInsideMinX_{0};
    //! Ignore points inside this box with max_x in map frame.
    double ignorePointsInsideMaxX_{0};
    //! Ignore points inside this box with min_y in map frame.
    double ignorePointsInsideMinY_{0};
    //! Ignore points inside this box with max_y in map frame.
    double ignorePointsInsideMaxY_{0};
    //! Ignore points inside this box with min_z in map frame.
    double ignorePointsInsideMinZ_{0};
    //! Ignore points inside this box with max_z in map frame.
    double ignorePointsInsideMaxZ_{0};

    //! Use VoxelGrid filter to cleanup pointcloud if true.
    bool applyVoxelGridFilter_{false};

    //! Sensor parameters.
    std::unordered_map<std::string, double> sensorParameters_;
  };

  ThreadSafeDataWrapper<Parameters> parameters_;

  //! Sensor parameters.
  std::unordered_map<std::string, double> sensorParameters_;

  //! Use VoxelGrid filter to cleanup pointcloud if true.
  //bool applyVoxelGridFilter_;

  //! Indicates if the requested tf transformation was available.
  bool firstTfAvailable_;
};

} /* namespace elevation_mapping */
