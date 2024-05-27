#include "plane_segmentation/ransac_segmentation.h"

namespace march_vision {

RansacSegmentation::RansacSegmentation(const RansacPlaneExtractorParameters& parameters) {
  setParameters(parameters);
  m_ransac.add_shape_factory<Plane>();
}

void RansacSegmentation::setParameters(const RansacPlaneExtractorParameters& parameters) {
  m_cgal_ransac_parameters.probability = parameters.probability;
  m_cgal_ransac_parameters.min_points = parameters.min_points;
  m_cgal_ransac_parameters.epsilon = parameters.epsilon / 3.0;  // CGAL ransac puts the inlier tolerance at 3 times epsilon
  m_cgal_ransac_parameters.cluster_epsilon = parameters.cluster_epsilon;
  m_cgal_ransac_parameters.normal_threshold = std::cos(parameters.normal_threshold * M_PI / 180.0);
}

void RansacSegmentation::detectPlanes(std::vector<PointWithNormal>& points_with_normal) {
  m_ransac.set_input(points_with_normal);
  m_ransac.detect(m_cgal_ransac_parameters);
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> RansacSegmentation::getPlaneParameters(Shape* shapePtr) {
  const auto* planePtr = static_cast<Plane*>(shapePtr);

  // Get Normal, pointing upwards
  Eigen::Vector3d normalVector(planePtr->plane_normal().x(), planePtr->plane_normal().y(), planePtr->plane_normal().z());
  if (normalVector.z() < 0.0) {
    normalVector = -normalVector;
  }

  // Project origin to get a point on the plane.
  const auto support = planePtr->projection({0.0, 0.0, 0.0});
  const Eigen::Vector3d supportVector(support.x(), support.y(), support.z());

  return {normalVector, supportVector};
}

}  // namespace march_vision
