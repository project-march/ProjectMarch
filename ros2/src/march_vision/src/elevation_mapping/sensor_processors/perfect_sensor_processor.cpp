/*
 * P.Fankhauser, M.Bloesch, and M.Hutter, 
 * "Probabilistic Terrain Mapping for Mobile Robots with Uncertain Localization",
 * in IEEE Robotics and Automation Letters (RA-L)
 *
 * MARCH functionality and ROS2 porting by Alexander Andonov
 */

#include "march_vision/elevation_mapping/sensor_processors/perfect_sensor_processor.hpp"

// PCL
#include <pcl/filters/filter.h>

// STD
#include <limits>
#include <string>
#include <vector>

#include "march_vision/elevation_mapping/point_XYZ_RGB_confidence_ratio.hpp"

namespace elevation_mapping {

/*!
 * Noiseless, perfect sensor.
 */

PerfectSensorProcessor::PerfectSensorProcessor(std::shared_ptr<rclcpp::Node>& nodeHandle,
                                              const SensorProcessorBase::GeneralParameters& generalParameters)
    : SensorProcessorBase(nodeHandle, generalParameters) {}

PerfectSensorProcessor::~PerfectSensorProcessor() = default;

bool PerfectSensorProcessor::readParameters(std::string& inputSourceName) {
  return SensorProcessorBase::readParameters(inputSourceName);
}

bool PerfectSensorProcessor::computeVariances(const PointCloudType::ConstPtr pointCloud,
                                              const Eigen::Matrix<double, 6, 6>& robotPoseCovariance, Eigen::VectorXf& variances) {
  variances.resize(pointCloud->size());

  // Projection vector (P).
  const Eigen::RowVector3f projectionVector = Eigen::RowVector3f::UnitZ();

  // Sensor Jacobian (J_s).
  const Eigen::RowVector3f sensorJacobian =
      projectionVector * (rotationMapToBase_.transposed() * rotationBaseToSensor_.transposed()).toImplementation().cast<float>();

  // Robot rotation covariance matrix (Sigma_q).
  const Eigen::Matrix3f rotationVariance = robotPoseCovariance.bottomRightCorner(3, 3).cast<float>();

  // Preparations for robot rotation Jacobian (J_q) to minimize computation for every point in point cloud.
  const Eigen::Matrix3f C_BM_transpose = rotationMapToBase_.transposed().toImplementation().cast<float>();
  const Eigen::RowVector3f P_mul_C_BM_transpose = projectionVector * C_BM_transpose;
  const Eigen::Matrix3f C_SB_transpose = rotationBaseToSensor_.transposed().toImplementation().cast<float>();
  const Eigen::Matrix3f B_r_BS_skew =
      kindr::getSkewMatrixFromVector(Eigen::Vector3f(translationBaseToSensorInBaseFrame_.toImplementation().cast<float>()));

  for (unsigned int i = 0; i < pointCloud->size(); ++i) {
    // For every point in point cloud.

    // Preparation.
    auto& point = pointCloud->points[i];
    Eigen::Vector3f pointVector(point.x, point.y, point.z);  // S_r_SP // NOLINT(cppcoreguidelines-pro-type-union-access)
    float heightVariance = 0.0;                              // sigma_p

    // Compute sensor covariance matrix (Sigma_S) with sensor model.
    float varianceNormal = 0.0;
    float varianceLateral = 0.0;
    Eigen::Matrix3f sensorVariance = Eigen::Matrix3f::Zero();
    sensorVariance.diagonal() << varianceLateral, varianceLateral, varianceNormal;

    // Robot rotation Jacobian (J_q).
    const Eigen::Matrix3f C_SB_transpose_times_S_r_SP_skew = kindr::getSkewMatrixFromVector(Eigen::Vector3f(C_SB_transpose * pointVector));
    const Eigen::RowVector3f rotationJacobian = P_mul_C_BM_transpose * (C_SB_transpose_times_S_r_SP_skew + B_r_BS_skew);

    // Measurement variance for map (error propagation law).
    heightVariance = rotationJacobian * rotationVariance * rotationJacobian.transpose();
    heightVariance += sensorJacobian * sensorVariance * sensorJacobian.transpose();

    // Copy to list.
    variances(i) = heightVariance;
  }

  return true;
}

}  // namespace elevation_mapping
