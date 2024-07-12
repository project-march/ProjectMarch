/*
 * P.Fankhauser, M.Bloesch, and M.Hutter, 
 * "Probabilistic Terrain Mapping for Mobile Robots with Uncertain Localization",
 * in IEEE Robotics and Automation Letters (RA-L)
 *
 * MARCH functionality and ROS2 porting by Alexander Andonov
 */

#include "elevation_mapping/input_sources/input_source_manager.hpp"
#include "elevation_mapping/elevation_mapping.hpp"

namespace elevation_mapping {

InputSourceManager::InputSourceManager(const std::shared_ptr<rclcpp::Node>& nodeHandle) : nodeHandle_(nodeHandle) {}

bool InputSourceManager::configureFromRos() {
  nodeHandle_->declare_parameter("inputs", std::vector<std::string>());

  // Configure the visualizations from a configuration stored on the parameter server.
  std::vector<std::string> inputSourcesConfiguration;
  if (!nodeHandle_->get_parameter("inputs", inputSourcesConfiguration)) {
    RCLCPP_WARN(nodeHandle_->get_logger(),
        "Could not load the input sources configuration from parameter\n "
        "%s, are you sure it was pushed to the parameter server? Assuming\n "
        "that you meant to leave it empty. Not subscribing to any inputs!\n", "inputs");
    return false;
  }
  
  return configure(inputSourcesConfiguration);  
}

bool InputSourceManager::configure(const std::vector<std::string>& config) {
  
  if (config.size() == 0) {  // Use Empty array as special case to explicitly configure no inputs.
    return true;
  }
 
  bool successfulConfiguration = true;
  std::set<std::string> subscribedTopics;
  SensorProcessorBase::GeneralParameters generalSensorProcessorConfig{nodeHandle_->get_parameter("robot_base_frame_id").as_string(),
                                                                      nodeHandle_->get_parameter("map_frame_id").as_string()};
  // Configure all input sources in the list.
  for (auto inputConfig : config) {
    // FIXME: fix namespace and subnode
    // return leading / -> rclcpp::expand_topic_or_service_name(sourceConfigurationName + "/" + inputConfig, nodeHandle_->get_name(), nodeHandle_->get_namespace()
    // auto subnode = nodeHandle_->create_sub_node(sourceConfigurationName + "/" + inputConfig);
    Input source = Input(nodeHandle_);

    bool configured = source.configure(inputConfig, generalSensorProcessorConfig);
    if (!configured) {
      successfulConfiguration = false;
      continue;
    }

    std::string subscribedTopic = source.getSubscribedTopic();
    bool topicIsUnique = subscribedTopics.insert(subscribedTopic).second;

    if (topicIsUnique) {
      sources_.push_back(std::move(source));
    } else {
      RCLCPP_WARN(nodeHandle_->get_logger(),
          "The input sources specification tried to subscribe to %s "
          "multiple times. Only subscribing once.",
          subscribedTopic.c_str());
      successfulConfiguration = false;
    }
  }

  return successfulConfiguration;
}

int InputSourceManager::getNumberOfSources() {
  return static_cast<int>(sources_.size());
}
}  // namespace elevation_mapping