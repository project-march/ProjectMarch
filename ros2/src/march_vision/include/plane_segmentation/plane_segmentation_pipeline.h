#pragma once

//#include "convex_plane_decomposition/GridMapPreprocessing.h"
#include "march_vision/planar_region.h"
#include "march_vision/Postprocessing.h"
#include "march_vision/contour_segmentation.h"
#include "march_vision/planar_image_segmentation.h"

namespace march_vision {

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
    planar_image_segmentation::PlanarImageSegmentationParameters PlanarImageSegmentationParameters;
    ransac_segmentation::RansacSegmentationParameters RansacSegmentationParameters;
    contour_segmentation::ContourSegmentationParameters ContourSegmentationParameters;
    // TODO: Remove if we're not going to use it
    PostprocessingParameters postprocessingParameters;
  };


  PlaneSegmentationPipeline(const Config& config);

  void update(grid_map::GridMap&& grid_map, const std::string& elevation_layer);

  /// Access to the pipeline result.
  PlanarTerrain& getPlanarTerrain() { return m_planar_terrain; }

  /// Fills in the resulting segmentation into a gridmap layer data.
  void getSegmentation(grid_map::GridMap::Matrix& segmentation) const;

 private:
  PlanarTerrain m_planar_terrain;

  // Pipeline
  //GridMapPreprocessing preprocessing_;
  planar_image_segmentation::PlanarImageSegmentation m_planar_image_segmentation;
  contour_segmentation::ContourSegmentation m_contour_segmentation;
  // TODO: Remove if we're not going to use it
  Postprocessing postprocessing_;

};

}  // namespace march_vision
