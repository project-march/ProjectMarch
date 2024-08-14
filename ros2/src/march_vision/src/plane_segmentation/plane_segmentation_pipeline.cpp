#include "march_vision/plane_segmentation_pipeline.h"

#include <opencv2/core/eigen.hpp>

namespace march_vision {

PlaneSegmentationPipeline::PlaneSegmentationPipeline(const Config& config)
    : m_planar_image_segmentation(config.PlanarImageSegmentationParameters, config.RansacSegmentationParameters),
      m_contour_segmentation(config.ContourSegmentationParameters),
      m_postprocessing(config.PostprocessingParameters) {}

// TODO: Add timers?
void PlaneSegmentationPipeline::update(grid_map::GridMap&& grid_map, const std::string& elevation_layer) {
  // Clear / Overwrite old result
  m_planar_terrain.planar_regions.clear();
  m_planar_terrain.grid_map = std::move(grid_map);

  //preprocessing_.preprocess(m_planar_terrain.gridMap, elevation_layer);

  m_planar_image_segmentation.runExtraction(m_planar_terrain.gridMap, elevation_layer);

  m_planar_terrain.planarRegions = m_contour_segmentation.extractPlanarRegions(m_planar_image_segmentation.getSegmentedPlanesMap());

  // Add binary map
  // TODO: Don't know if we're going to use this
  const std::string planeClassificationLayer{"plane_classification"};
  m_planar_terrain.gridMap.add(planeClassificationLayer);
  auto& traversabilityMask = m_planar_terrain.gridMap.get(planeClassificationLayer);
  cv::cv2eigen(m_planar_image_segmentation.getBinaryLabeledImage(), traversabilityMask);

  m_postprocessing.postprocess(m_planar_terrain, elevation_layer, planeClassificationLayer);
}

void PlaneSegmentationPipeline::getSegmentation(grid_map::GridMap::Matrix& segmentation) const {
  cv::cv2eigen(m_planar_image_segmentation.getSegmentedPlanesMap().labeled_image, segmentation);
}

}  // namespace march_vision