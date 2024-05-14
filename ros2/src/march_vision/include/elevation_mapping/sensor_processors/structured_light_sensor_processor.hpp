/*
 * P.Fankhauser, M.Bloesch, and M.Hutter, 
 * "Probabilistic Terrain Mapping for Mobile Robots with Uncertain Localization",
 * in IEEE Robotics and Automation Letters (RA-L)
 *
 * MARCH functionality and ROS2 porting by Alexander Andonov
 */

#pragma once

#include "march_vision/elevation_mapping/sensor_processors/sensor_processor_base.hpp"
#include "march_vision/elevation_mapping/point_XYZ_RGB_confidence_ratio.hpp"

namespace elevation_mapping {

/*!
 * Sensor processor for StructuredLight-type (PrimeSense) structured light sensors.
 * The RealSense 435i cams fall in this category
 */
class StructuredLightSensorProcessor : public SensorProcessorBase {
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  StructuredLightSensorProcessor(std::shared_ptr<rclcpp::Node>& nodeHandle,
    const SensorProcessorBase::GeneralParameters& generalParameters);

  /*!
   * Destructor.
   */
  ~StructuredLightSensorProcessor() override;

 private:
  /*!
   * Reads and verifies the parameters.
   * @return true if successful.
   */
  bool readParameters(std::string& inputSourceName) override;

  /*!
   * Computes the elevation map height variances for each point in a point cloud with the
   * sensor model and the robot pose covariance.
   * @param[in] pointCloud the point cloud for which the variances are computed.
   * @param[in] robotPoseCovariance the robot pose covariance matrix.
   * @param[out] variances the elevation map height variances.
   * @return true if successful.
   */
  bool computeVariances(const PointCloudType::ConstPtr pointCloud, const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
                        Eigen::VectorXf& variances) override;

  /*!
   * Cuts off points that are not within the cutoff interval
   * @param pointCloud the point cloud to filter.
   * @return true if successful.
   */
  bool filterPointCloudSensorType(const PointCloudType::Ptr pointCloud) override;
};
} /* namespace elevation_mapping */
