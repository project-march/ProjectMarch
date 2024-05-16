#include "march_vision/plane_segmentation/plane_segmentation_pipeline.h"

#include <opencv2/core/eigen.hpp>

namespace plane_segmentation {

PlaneSegmentationPipeline::PlaneSegmentationPipeline(std::shared_ptr<rclcpp::Node>& node_handle, const Config& config)
    : m_node_handle(node_handle),
      m_planar_image_segmentation(config.PlanarImageSegmentationParameters, config.RansacSegmentationParameters),
      m_contour_segmentation(config.ContourSegmentationParameters),
      m_postprocessing(config.PostprocessingParameters) 
      {
        RCLCPP_INFO(m_node_handle->get_logger(), "Plane segmentation node started.");
        bool is_parameters_read = readParameters(m_node_handle);
        if(is_parameters_read){
          // TODO: Setup subscribers and publishers.7
          // Input: filtered/postprocessed elevation map
          // elevation_mapping_node_sub
          // 
          // RViz visualizations
          // boundaries_pub
          // insets_pub

          // Output: planar terrain
          // regions_pub
        }
        
      }

// TODO: Add timers?
void PlaneSegmentationPipeline::update(grid_map::GridMap&& grid_map, const std::string& elevation_layer) {
  // Clear/overwrite old result
  m_planar_terrain.planar_regions.clear();
  m_planar_terrain.grid_map = std::move(grid_map);

  //preprocessing_.preprocess(m_planar_terrain.gridMap, elevation_layer);

  m_planar_image_segmentation.runExtraction(m_planar_terrain.gridMap, elevation_layer);

  m_planar_terrain.planarRegions = m_contour_segmentation.extractPlanarRegions(m_planar_image_segmentation.getSegmentedPlanesMap());

  // TODO: Don't know if we're going to use this
  m_postprocessing.postprocess(m_planar_terrain, elevation_layer, planeClassificationLayer);
}

void PlaneSegmentationPipeline::getSegmentation(grid_map::GridMap::Matrix& segmentation) const {
  cv::cv2eigen(m_planar_image_segmentation.getSegmentedPlanesMap().labeled_image, segmentation);
}

// TODO: Add a function to read parameters from a file and maybe refactor node and its ROS functionalities
// bool PlaneSegmentationPipeline::readParameters(){

//   Config config;
// }

bool PlaneSegmentationPipeline::readParameters(){

  m_node_handle->declare_parameter("plane_segmentation_parameters");
  m_node_handle->declare_parameter("plane_segmentation_parameters");
  m_node_handle->declare_parameter("plane_segmentation_parameters");
  m_node_handle->declare_parameter("plane_segmentation_parameters");
  m_node_handle->declare_parameter("plane_segmentation_parameters");
  m_node_handle->declare_parameter("plane_segmentation_parameters");

}





}  // namespace plane_segmentation