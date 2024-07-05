/*
 * P.Fankhauser, M.Bloesch, and M.Hutter, 
 * "Probabilistic Terrain Mapping for Mobile Robots with Uncertain Localization",
 * in IEEE Robotics and Automation Letters (RA-L)
 *
 * MARCH functionality and ROS2 porting by Alexander Andonov
 */

#include "elevation_mapping/input_sources/input.hpp"
#include "elevation_mapping/sensor_processors/perfect_sensor_processor.hpp"
#include "elevation_mapping/sensor_processors/structured_light_sensor_processor.hpp"

namespace elevation_mapping {

Input::Input(std::shared_ptr<rclcpp::Node> nh) : nodeHandle_(nh) {}

bool Input::configure(std::string& inputSourceName, const std::string& sourceConfigurationName, const SensorProcessorBase::GeneralParameters& generalSensorProcessorParameters) {
  // TODO: make nicer add checkes
  // TODO: sourceConfigurationName not used
  RCLCPP_INFO(nodeHandle_->get_logger(), "Configuring input source:  %s.", inputSourceName.c_str());
  Parameters parameters;

  std::string sensorProcessorType;

  nodeHandle_->declare_parameter(sourceConfigurationName + "." + inputSourceName + ".type");
  nodeHandle_->declare_parameter(sourceConfigurationName + "." + inputSourceName + ".topic");
  nodeHandle_->declare_parameter(sourceConfigurationName + "." + inputSourceName + ".queue_size");
  nodeHandle_->declare_parameter(sourceConfigurationName + "." + inputSourceName + ".publish_on_update");
  nodeHandle_->declare_parameter(sourceConfigurationName + "." + inputSourceName + ".sensor_processor.type");

  if (!nodeHandle_->get_parameter(sourceConfigurationName + "." + inputSourceName + ".type", parameters.type_)){
    RCLCPP_ERROR(nodeHandle_->get_logger(), "Could not configure input source %s because no type was given.", inputSourceName.c_str());
  } else RCLCPP_INFO(nodeHandle_->get_logger(), "Input source type: %s", parameters.type_.c_str());

  if (!nodeHandle_->get_parameter(sourceConfigurationName + "." + inputSourceName + ".topic", parameters.topic_)){
    RCLCPP_ERROR(nodeHandle_->get_logger(), "Could not configure input source %s because no topic was given.", inputSourceName.c_str());
  }else RCLCPP_INFO(nodeHandle_->get_logger(), "Input source topic: %s", parameters.topic_.c_str());

  if (!nodeHandle_->get_parameter(sourceConfigurationName + "." + inputSourceName + ".queue_size", parameters.queueSize_)){
    RCLCPP_ERROR(nodeHandle_->get_logger(), "Could not configure input source %s because no queue_size was given.", inputSourceName.c_str());
  }else RCLCPP_INFO(nodeHandle_->get_logger(), "Input source queue_size: %d", parameters.queueSize_);

  if (!nodeHandle_->get_parameter(sourceConfigurationName + "." + inputSourceName + ".publish_on_update", parameters.publishOnUpdate_)){
    RCLCPP_ERROR(nodeHandle_->get_logger(), "Could not configure input source %s because no publish_on_update was given.", inputSourceName.c_str());
  }else RCLCPP_INFO(nodeHandle_->get_logger(), "Input source publish_on_update: %s", parameters.publishOnUpdate_ ? "true" : "false");

  if (!nodeHandle_->get_parameter(sourceConfigurationName + "." + inputSourceName + ".sensor_processor.type", sensorProcessorType)){
    RCLCPP_ERROR(nodeHandle_->get_logger(), "Could not configure input source %s because no sensor_processor was given.", inputSourceName.c_str());
  }else RCLCPP_INFO(nodeHandle_->get_logger(), "Input source sensor_processor: %s", sensorProcessorType.c_str());

  parameters.name_ = inputSourceName;
  
  parameters_.setData(parameters);
 
  // SensorProcessor
  if (!configureSensorProcessor(inputSourceName, sensorProcessorType, generalSensorProcessorParameters)) {
    return false;
  }

  RCLCPP_DEBUG(nodeHandle_->get_logger(), "Configured %s:%s @ %s (publishing_on_update: %s), using %s to process data.\n", parameters.type_.c_str(), parameters.name_.c_str(),
            rclcpp::expand_topic_or_service_name(parameters.topic_, nodeHandle_->get_name(), nodeHandle_->get_namespace()).c_str(), parameters.publishOnUpdate_ ? "true" : "false", sensorProcessorType.c_str());
  return true;
}

std::string Input::getSubscribedTopic() const {
  const Parameters parameters{parameters_.getData()};
  // FIXME: 
  return rclcpp::expand_topic_or_service_name(parameters.topic_, nodeHandle_->get_name(), nodeHandle_->get_namespace());
}

bool Input::configureSensorProcessor(std::string& inputSourceName, const std::string& sensorType,
                                     const SensorProcessorBase::GeneralParameters& generalSensorProcessorParameters) {
  
  if (sensorType == "structured_light") {
    sensorProcessor_.reset(new StructuredLightSensorProcessor(nodeHandle_, generalSensorProcessorParameters));
  } else if (sensorType == "perfect") {
    sensorProcessor_.reset(new PerfectSensorProcessor(nodeHandle_, generalSensorProcessorParameters));
  } else {
    RCLCPP_ERROR(nodeHandle_->get_logger(), "The sensor type %s is not available.", sensorType.c_str());
    return false;
  }

  return sensorProcessor_->readParameters(inputSourceName);
}

}  // namespace elevation_mapping
