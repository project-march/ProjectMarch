#include "march_vision/plane_segmentation/contour_segmentation.h"

// TODO: Add geometry utils or add them in this
#include <march_vision/plane_segmentation/geometry_utils.h>
#include <opencv2/imgproc.hpp>

namespace plane_segmentation {

namespace {

// Helper function that upsamples a single column. Writes into a target that is already allocated with the right size
void upSampleColumn(const cv::Mat& column, cv::Mat& target, int col, int rows, int upsampled_rows) {

  for (int row = 0; row < rows; ++row) {
    const auto value = column.at<float>(row);
    if (row == 0) {
      target.at<float>(0, col) = value;
      target.at<float>(1, col) = value;
    } else if (row + 1 == rows) {
      target.at<float>(upsampled_rows - 2, col) = value;
      target.at<float>(upsampled_rows - 1, col) = value;
    } else {
      const int target_row = 2 + 3 * (row - 1);
      target.at<float>(target_row, col) = value;
      target.at<float>(target_row + 1, col) = value;
      target.at<float>(target_row + 2, col) = value;
    }
  }
}
}  // namespace

cv::Mat upSample(const cv::Mat& image) {
  const int rows = image.rows;
  const int cols = image.cols;
  assert(rows >= 2);
  assert(cols >= 2);
  const int upsampled_rows = 4 + 3 * (rows - 2);
  const int upsampled_cols = 4 + 3 * (cols - 2);

  cv::Mat result(upsampled_rows, upsampled_cols, image.type());

  for (int col = 0; col < cols; ++col) {
    const auto& column = image.col(col);
    if (col == 0) {
      upSampleColumn(column, result, 0, rows, upsampled_rows);
      result.col(0).copyTo(result.col(1));
    } else if (col + 1 == cols) {
      upSampleColumn(column, result, upsampled_cols - 2, rows, upsampled_rows);
      result.col(upsampled_cols - 2).copyTo(result.col(upsampled_cols - 1));
    } else {
      const int target_col = 2 + 3 * (col - 1);
      upSampleColumn(column, result, target_col, rows, upsampled_rows);
      result.col(target_col).copyTo(result.col(target_col + 1));
      result.col(target_col + 1).copyTo(result.col(target_col + 2));
    }
  }
  return result;
}

SegmentedPlanesMap upSample(const SegmentedPlanesMap& map_in) {
  
  SegmentedPlanesMap map_out;
  map_out.label_plane_parameters = map_in.label_plane_parameters;
  map_out.labeled_image = upSample(map_in.labeled_image);
  map_out.resolution = map_in.resolution / 3.0;
  map_out.map_origin = map_in.map_origin;
  map_out.highest_label = map_in.highest_label;
  return map_out;
}

ContourSegmentation::ContourSegmentation(const ContourSegmentationParameters& parameters)
    : m_contour_parameters(parameters), m_binary_image(cv::Size(0, 0), CV_8UC1) {
  {
    int erosion_size = 1;  // single sided length of the kernel
    int erosion_type = cv::MORPH_CROSS;
    m_inset_kernel = cv::getStructuringElement(erosion_type, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1));
  }
  {
    int erosion_size = 1 + m_contour_parameters.margin_size;  // single sided length of the kernel
    int erosion_type = cv::MORPH_ELLIPSE;
    m_margin_kernel = cv::getStructuringElement(erosion_type, cv::Size(2 * erosion_size + 1, 2 * erosionSize + 1));
  }
}


std::vector<PlanarRegion> ContourSegmentation::extractPlanarRegions(const SegmentedPlanesMap& segmented_planes_map) {
  
  const auto upsampled_map = upSample(segmented_planes_map);

  std::vector<PlanarRegion> planar_regions;
  planar_regions.reserve(upsampled_map.highest_label + 1);
  for (const auto& label_plane : upsampled_map.label_plane_parameters) {
    const int label = label_plane.first;
    m_binary_image = upsampled_map.labeled_image == label;

    // Try with safety margin
    cv::erode(m_binary_image, m_binary_image, m_margin_kernel, cv::Point(-1,-1), 1, cv::BORDER_REPLICATE);
    auto boundaries_and_insets = contour_extraction::extractBoundaryAndInset(m_binary_image, m_inset_kernel);

    // If safety margin makes the region disappear -> try without
    if (boundaries_and_insets.empty()) {
      m_binary_image = upsampled_map.labeled_image == label;
      // still 1 pixel erosion to remove the growth after upsampling
      cv::erode(m_binary_image, m_binary_image, m_inset_kernel, cv::Point(-1,-1), 1, cv::BORDER_REPLICATE);
      boundaries_and_insets = contour_extraction::extractBoundaryAndInset(m_binary_image, m_inset_kernel);
    }

    
    // TODO: Remove this since we use only local map frame
    const auto plane_parameters = getTransformLocalToGlobal(label_plane.second);
    for (auto& boundary_and_inset : boundaries_and_insets) {
      // Transform points from pixel space to local terrain frame
      transformInPlace(boundary_and_inset, [&](CgalPoint2d& point) {
        auto point_in_world = pixelToWorldFrame(point, upSampledMap.resolution, upsampled_map.map_origin);
        point = projectToPlaneAlongGravity(point_in_world, plane_parameters);
      });

      PlanarRegion planar_region;
      planar_region.boundary_with_inset = std::move(boundary_and_inset);
      planar_region.transform_plan_to_world = plane_parameters;
      planar_region.bbox2d = planar_region.boundary_with_inset.boundary.outer_boundary().bbox();
      planar_region.push_back(std::move(planar_region));
    }
  }
  return planar_regions;
}

std::vector<BoundaryWithInset> extractBoundaryAndInset(cv::Mat& binary_image, const cv::Mat& erosion_kernel) {
  // Get boundary
  std::vector<CgalPolygonWithHoles2d> boundaries = extractPolygonsFromBinaryImage(binary_image);

  // Erode
  cv::erode(binary_image, binary_image, erosion_kernel, cv::Point(-1,-1), 1, cv::BORDER_REPLICATE);

  // Erosion of the edge of the map
  binary_image.row(0) = 0;
  binary_image.row(binary_image.rows - 1) = 0;
  binary_image.col(0) = 0;
  binary_image.col(binary_image.cols - 1) = 0;

  // Get insets
  std::vector<CgalPolygonWithHoles2d> insets = extractPolygonsFromBinaryImage(binary_image);

  // Associate boundaries with insets
  std::vector<BoundaryWithInset> boundaries_with_insets;
  for (const auto& boundary : boundaries) {
    std::vector<CgalPolygonWithHoles2d> assigned_insets;
    for (const auto& inset : insets) {
      if (isInside(inset.outer_boundary().vertex(0), boundary)) {
        assigned_insets.push_back(inset);
      }
    }

    if (!assigned_insets.empty()) {
      BoundaryWithInset boundary_with_inset;
      boundary_with_inset.boundary = boundary;
      boundary_with_inset.insets = assigned_insets;
      boundaries_with_insets.push_back(std::move(boundary_with_inset));
    }
  }
  return boundaries_with_insets;
}
std::vector<CgalPolygonWithHoles2d> extractPolygonsFromBinaryImage(const cv::Mat& binary_image) {
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;  // [Next, Previous, First_Child, Parent]
  auto isOuterContour = [](const cv::Vec4i& hierarchyVector) {
    return hierarchy_vector[3] < 0;  // no parent
  };

  cv::findContours(binary_image, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

  std::vector<CgalPolygonWithHoles2d> plane_polygons;
  for (int i = 0; i < contours.size(); i++) {
    if (isOuterContour(hierarchy[i]) && contours[i].size() > 1) {
      CgalPolygonWithHoles2d polygon;
      polygon.outer_boundary() = cgalPolygonFromOpenCv(contours[i]);

      // Add children as holes
      int child_index = hierarchy[i][2];  // First child
      while (child_index > 0) {
        polygon.add_hole(cgalPolygonFromOpenCv(contours[child_index]));
        child_index = hierarchy[child_index][0];  // Next child
      }
      plane_polygons.push_back(std::move(polygon));
    }
  }
  return plane_polygons;
}

CgalPolygon2d cgalPolygonFromOpenCv(const std::vector<cv::Point>& openCV_polygon) {
  CgalPolygon2d polygon;
  polygon.container().reserve(openCV_polygon.size());
  for (const auto& point : openCV_polygon) {
    polygon.container().emplace_back(point.x, point.y);
  }
  return polygon;
}

CgalPoint2d pixelToWorldFrame(const CgalPoint2d& pixelspace_cgal_point2d, double resolution, const Eigen::Vector2d& map_offset) {
  // Notice the transpose of x and y!
  return {map_offset.x() - resolution * pixelspace_cgal_point2d.y(), map_offset.y() - resolution * pixelspace_cgal_point2d.x()};
}

}  // namespace plane_segmentation
