#pragma once

//#include "plane_segmentation/grid_map_preprocessing.h"
#include "plane_segmentation/planar_region.h"
// TODO: Remove if we're not going to use it
// #include "march_vision/postprocessing.h"
#include "plane_segmentation/contour_segmentation.h"
#include "plane_segmentation/planar_image_segmentation.h"

namespace plane_segmentation {

/**
 * Input:
 *   - Elevation map
 * Output:
 *   - Planar terrain (planes + filtered elevation map)
 *   - Segmented elevation map
 */
class PlaneSegmentationPipeline {

 public:

  struct Config {
    
    PlanarImageSegmentation::PlanarImageSegmentationParameters PlanarImageSegmentationParameters;
    RansacSegmentation::RansacSegmentationParameters RansacSegmentationParameters;
    ContourSegmentation::ContourSegmentationParameters ContourSegmentationParameters;
    // TODO: Remove if we're not going to use it
    PostprocessingParameters postprocessingParameters;
  };


  PlaneSegmentationPipeline(const Config& config);
  bool readParameters() const;
  void update(grid_map::GridMap&& grid_map, const std::string& elevation_layer);

  /// Access to the pipeline result.
  PlanarTerrain& getPlanarTerrain() { return m_planar_terrain; }

  /// Fills in the resulting segmentation into a gridmap layer data.
  void getSegmentation(grid_map::GridMap::Matrix& segmentation) const;

 private:
  PlanarTerrain m_planar_terrain;

  // Pipeline
  // GridMapPreprocessing preprocessing_;
  PlanarImageSegmentation::PlanarImageSegmentation m_planar_image_segmentation;
  ContourSegmentation::ContourSegmentation m_contour_segmentation;
  // TODO: Remove if we're not going to use it
  Postprocessing postprocessing_;

};

}  // namespace plane_segmentation
