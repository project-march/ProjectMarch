/*
 * P.Fankhauser, M.Bloesch, and M.Hutter, 
 * "Probabilistic Terrain Mapping for Mobile Robots with Uncertain Localization",
 * in IEEE Robotics and Automation Letters (RA-L)
 *
 * MARCH functionality and ROS2 porting by Alexander Andonov
 */

#include <limits>
#include <string>
#include <vector>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/sensor_processors/StructuredLightSensorProcessor.hpp"

namespace elevation_mapping {

/*! StructuredLight-type (structured light) sensor model:
 * standardDeviationInNormalDirection = sensorModelNormalFactorA_ + sensorModelNormalFactorB_ * (measurementDistance -
 * sensorModelNormalFactorC_)^2; standardDeviationInLateralDirection = sensorModelLateralFactor_ * measurementDistance Taken from: Nguyen,
 * C. V., Izadi, S., & Lovell, D., Modeling Kinect Sensor Noise for Improved 3D Reconstruction and Tracking, 2012.
 */

StructuredLightSensorProcessor::StructuredLightSensorProcessor(std::shared_ptr<rclcpp::Node>& nodeHandle,
                                                               const SensorProcessorBase::GeneralParameters& generalParameters)
    : SensorProcessorBase(nodeHandle, generalParameters) {}

StructuredLightSensorProcessor::~StructuredLightSensorProcessor() = default;

bool StructuredLightSensorProcessor::readParameters(std::string& inputSourceName) {
  SensorProcessorBase::readParameters(inputSourceName);
  nodeHandle_->declare_parameter(inputSourceName + ".sensor_processor.normal_factor_a", rclcpp::ParameterValue(0.0));
  nodeHandle_->declare_parameter(inputSourceName + ".sensor_processor.normal_factor_b", rclcpp::ParameterValue(0.0));
  nodeHandle_->declare_parameter(inputSourceName + ".sensor_processor.normal_factor_c", rclcpp::ParameterValue(0.0));
  nodeHandle_->declare_parameter(inputSourceName + ".sensor_processor.normal_factor_d", rclcpp::ParameterValue(0.0));
  nodeHandle_->declare_parameter(inputSourceName + ".sensor_processor.normal_factor_e", rclcpp::ParameterValue(0.0));
  nodeHandle_->declare_parameter(inputSourceName + ".sensor_processor.lateral_factor", rclcpp::ParameterValue(0.0));
  nodeHandle_->declare_parameter(inputSourceName + ".sensor_processor.cutoff_min_depth", rclcpp::ParameterValue(std::numeric_limits<double>::min()));
  nodeHandle_->declare_parameter(inputSourceName + ".sensor_processor.cutoff_max_depth", rclcpp::ParameterValue(std::numeric_limits<double>::max()));
  
  nodeHandle_->get_parameter(inputSourceName + ".sensor_processor.normal_factor_a", sensorParameters_["normal_factor_a"]);
  nodeHandle_->get_parameter(inputSourceName + ".sensor_processor.normal_factor_b", sensorParameters_["normal_factor_b"]);
  nodeHandle_->get_parameter(inputSourceName + ".sensor_processor.normal_factor_c", sensorParameters_["normal_factor_c"]);
  nodeHandle_->get_parameter(inputSourceName + ".sensor_processor.normal_factor_d", sensorParameters_["normal_factor_d"]);
  nodeHandle_->get_parameter(inputSourceName + ".sensor_processor.normal_factor_e", sensorParameters_["normal_factor_e"]);
  nodeHandle_->get_parameter(inputSourceName + ".sensor_processor.lateral_factor", sensorParameters_["lateral_factor"]);
  nodeHandle_->get_parameter(inputSourceName + ".sensor_processor.cutoff_min_depth", sensorParameters_["cutoff_min_depth"]);
  nodeHandle_->get_parameter(inputSourceName + ".sensor_processor.cutoff_max_depth", sensorParameters_["cutoff_max_depth"]);

  // nodeHandle_.param("sensor_processor/normal_factor_a", sensorParameters_["normal_factor_a"], 0.0);
  // nodeHandle_.param("sensor_processor/normal_factor_b", sensorParameters_["normal_factor_b"], 0.0);
  // nodeHandle_.param("sensor_processor/normal_factor_c", sensorParameters_["normal_factor_c"], 0.0);
  // nodeHandle_.param("sensor_processor/normal_factor_d", sensorParameters_["normal_factor_d"], 0.0);
  // nodeHandle_.param("sensor_processor/normal_factor_e", sensorParameters_["normal_factor_e"], 0.0);
  // nodeHandle_.param("sensor_processor/lateral_factor", sensorParameters_["lateral_factor"], 0.0);
  // nodeHandle_.param("sensor_processor/cutoff_min_depth", sensorParameters_["cutoff_min_depth"], std::numeric_limits<double>::min());
  // nodeHandle_.param("sensor_processor/cutoff_max_depth", sensorParameters_["cutoff_max_depth"], std::numeric_limits<double>::max());
  return true;
}

bool StructuredLightSensorProcessor::computeVariances(const PointCloudType::ConstPtr pointCloud,
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
  const float epsilon = std::numeric_limits<float>::min();

  for (unsigned int i = 0; i < pointCloud->size(); ++i) {
    // For every point in point cloud.

    // Preparation.
    auto& point = pointCloud->points[i];
    const float& confidenceRatio = point.confidence_ratio;
    Eigen::Vector3f pointVector(point.x, point.y, point.z);  // S_r_SP // NOLINT(cppcoreguidelines-pro-type-union-access)

    // Measurement distance.
    const float measurementDistance = pointVector.z();

    // Compute sensor covariance matrix (Sigma_S) with sensor model.
    const float deviationNormal =
        sensorParameters_.at("normal_factor_a") +
        sensorParameters_.at("normal_factor_b") * (measurementDistance - sensorParameters_.at("normal_factor_c")) *
            (measurementDistance - sensorParameters_.at("normal_factor_c")) +
        sensorParameters_.at("normal_factor_d") * pow(measurementDistance, sensorParameters_.at("normal_factor_e"));
    const float varianceNormal = deviationNormal * deviationNormal;
    const float deviationLateral = sensorParameters_.at("lateral_factor") * measurementDistance;
    const float varianceLateral = deviationLateral * deviationLateral;
    Eigen::Matrix3f sensorVariance = Eigen::Matrix3f::Zero();
    sensorVariance.diagonal() << varianceLateral, varianceLateral, varianceNormal;

    // Robot rotation Jacobian (J_q).
    const Eigen::Matrix3f C_SB_transpose_times_S_r_SP_skew = kindr::getSkewMatrixFromVector(Eigen::Vector3f(C_SB_transpose * pointVector));
    const Eigen::RowVector3f rotationJacobian = P_mul_C_BM_transpose * (C_SB_transpose_times_S_r_SP_skew + B_r_BS_skew);

    // Measurement variance for map (error propagation law).
    float heightVariance = 0.0;  // sigma_p
    heightVariance = rotationJacobian * rotationVariance * rotationJacobian.transpose();
    // Scale the sensor variance by the inverse, squared confidence ratio
    heightVariance +=
        static_cast<float>(sensorJacobian * sensorVariance * sensorJacobian.transpose()) / (epsilon + confidenceRatio * confidenceRatio);

    // Copy to list.
    variances(i) = heightVariance;
  }

  return true;
}

bool StructuredLightSensorProcessor::filterPointCloudSensorType(const PointCloudType::Ptr pointCloud) {
  pcl::PassThrough<pcl::PointXYZRGBConfidenceRatio> passThroughFilter;
  PointCloudType tempPointCloud;

  // cutoff points with z values
  passThroughFilter.setInputCloud(pointCloud);
  passThroughFilter.setFilterFieldName("z");
  passThroughFilter.setFilterLimits(sensorParameters_.at("cutoff_min_depth"), sensorParameters_.at("cutoff_max_depth"));
  passThroughFilter.filter(tempPointCloud);
  pointCloud->swap(tempPointCloud);

  return true;
}

}  // namespace elevation_mapping
