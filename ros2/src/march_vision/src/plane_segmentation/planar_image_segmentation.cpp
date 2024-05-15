#include "march_vision/plane_segmentation/planar_image_segmentation.h"

#include <chrono>
#include <cmath>
#include <iostream>

#include <Eigen/Eigenvalues>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <grid_map_core/grid_map_core.hpp>

namespace plane_segmentation {

namespace {

    std::pair<Eigen::Vector3d, double> normalAndErrorFromCovariance(
        int num_point, const Eigen::Vector3d& mean, const Eigen::Matrix3d& sum_squared)
    {
        const Eigen::Matrix3d covariance_matrix = sum_squared / num_point - mean * mean.transpose();

        // Eigenvalues are ordered small to large.
        // Worst case bound for zero eigenvalue from :
        // https://eigen.tuxfamily.org/dox/classEigen_1_1SelfAdjointEigenSolver.html
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
        solver.computeDirect(covariance_matrix, Eigen::DecompositionOptions::ComputeEigenvectors);
        if (solver.eigenvalues()(1) > 1e-8) {
            Eigen::Vector3d unitary_normal_vector = solver.eigenvectors().col(0);

            // Checks direction of the normal vector and flip the sign upwards
            if (unitary_normal_vector.z() < 0.0) {
                unitary_normal_vector = -unitary_normal_vector;
            }
            double square_error = (solver.eigenvalues()(0) > 0.0) ? solver.eigenvalues()(0) : 0.0;
            return { unitary_normal_vector, square_error };
        } else { // If second eigenvalue is zero, the normal is not defined.
            return { Eigen::Vector3d::UnitZ(), 1e30 };
        }
    }

} // namespace

PlanarImageSegmentation::PlanarImageSegmentation(const PlanarImageSegmentationParameters& parameters,
    const ransac_segmentation::RansacSegmentationParameters& ransac_parameters)
    : m_parameters(parameters)
    , m_ransac_parameters(ransac_parameters)
{
}

void PlanarImageSegmentation::runExtraction(const grid_map::GridMap& map, const std::string& layer_height)
{

    m_map = &map;
    m_elevation_layer = layer_height;
    m_map_rows = m_map->getSize()[0];
    m_segmented_planes_map.resolution = m_map->getResolution();
    m_map->getPosition(Eigen::Vector2i(0, 0), m_segmented_planes_map.map_origin);

    m_segmented_planes_map.highest_label = -1;
    m_segmented_planes_map.label_plane_parameters.clear();
    const auto& map_size = m_map->getSize();
    m_binary_image_patch
        = cv::Mat(mapSize(0), mapSize(1), CV_8U, 0.0); // Zero initialize to set untouched pixels to not planar;
    // Need a buffer of at least the linear size of the image. But there's no need to shrink if the buffer is already bigger.
    const int linear_map_size = mapSize(0) * mapSize(1);

    if (m_surface_normals.size() < linear_map_size) {
        m_surface_normals.reserve(linear_map_size);
        std::fill_n(m_surface_normals.begin(), linear_map_size, Eigen::Vector3d::Zero());
    }

    runSlidingWindowDetector();
    runSegmentation();
    extractPlaneParametersFromLabeledImage();

    // Get classification from segmentation to account for unassigned points.
    m_binary_image_patch = m_segmented_planes_map.labeled_image > 0;
    m_binary_image_patch.setTo(1, m_binary_image_patch == 255);
}

std::pair<Eigen::Vector3d, double> PlanarImageSegmentation::computeNormalAndErrorForWindow(
    const Eigen::MatrixXf& window_data) const
{
    // Get surrounding data.
    size_t n_points = 0;
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    Eigen::Matrix3d sum_squared = Eigen::Matrix3d::Zero();

    for (int kernel_col = 0; kernel_col < m_parameters.kernel_size; ++kernel_col) {
        for (int kernel_row = 0; kernel_row < m_parameters.kernel_size; ++kernel_row) {
            float height = window_data(kernel_row, kernel_col);
            if (!std::isfinite(height)) {
                continue;
            }
            // No need to account for map offset; will substract the mean anyway.
            Eigen::Vector3d point { -kernel_row * m_segmented_planes_map.resolution,
                -kernel_col * m_segmented_planes_map.resolution, height };
            n_points++;
            sum += point;
            sum_squared.noalias() += point * point.transpose();
        }
    }

    if (n_points < 3) {
        // not enough points to get normal direction
        return { Eigen::Vector3d::UnitZ(), 1e30 };
    } else {
        const Eigen::Vector3d mean = sum / n_points;
        return normalAndErrorFromCovariance(n_points, mean, sum_squared);
    }
}

bool PlanarImageSegmentation::isLocallyPlanar(const Eigen::Vector3d& local_normal, double mean_squared_error) const {

    const double threshold_squared
        = m_parameters.plane_patch_error_threshold * m_parameters.plane_patch_error_threshold;

    const double normal_dot_product = local_normal.z();
    return (mean_squared_error < threshold_squared && normal_dot_product > m_parameters.local_plane_inclination_threshold);
}

void PlanarImageSegmentation::runSlidingWindowDetector() {

    grid_map::SlidingWindowIterator window_iterator(
        *m_map, m_elevation_layer, grid_map::SlidingWindowIterator::EdgeHandling::EMPTY, m_parameters.kernel_size);
    const int kernel_middle = (m_parameters.kernel_size - 1) / 2;

    for (; !window_iterator.isPastEnd(); ++window_iterator) {

        grid_map::Index index = *window_iterator;
        Eigen::MatrixXf window_data = window_iterator.getData();
        const auto middle_value = window_data(kernel_middle, kernel_middle);

        if (!std::isfinite(middle_value)) {
            m_binary_image_patch.at<bool>(index.x(), index.y()) = false;
        } else {
            Eigen::Vector3d n;
            double mean_squared_error;
            std::tie(n, mean_squared_error) = computeNormalAndErrorForWindow(window_data);

            m_surface_normals[getLinearIndex(index.x(), index.y())] = n;
            m_binary_image_patch.at<bool>(index.x(), index.y()) = isLocallyPlanar(n, mean_squared_error);
        }
    }

    // TODO: Decide if I'm going to leave the opening filter here or not
    // opening filter
    if (m_parameters.planarity_opening_filter > 0) {
        const int opening_kernel_size = 2 * m_parameters.planarity_opening_filter + 1;
        const int opening_kernel_type = cv::MORPH_CROSS;
        const auto m_kernel
            = cv::getStructuringElement(opening_kernel_type, cv::Size(opening_kernel_size, opening_kernel_size));
        cv::morphologyEx(m_binary_image_patch, m_binary_image_patch, cv::MORPH_OPEN, m_kernel, cv::Point(-1, -1), 1,
            cv::BORDER_REPLICATE);
    }
}

void PlanarImageSegmentation::runSegmentation()
{
    int number_of_label = cv::connectedComponents(
        m_binary_image_patch, m_segmented_planes_map.labeled_image, m_parameters.connectivity, CV_32S);
    m_segmented_planes_map.highestLabel = number_of_label - 1; // Labels are [0, N-1]
}

void PlanarImageSegmentation::extractPlaneParametersFromLabeledImage()
{
    const int number_of_extracted_planes_without_refinement
        = m_segmented_planes_map.highestLabel; // Make local copy. The highestLabel is incremented inside the loop

    // Reserve a workvector that is reused between processing labels
    m_points_with_normal.reserve(m_segmented_planes_map.labeled_image.rows * m_segmented_planes_map.labeled_image.cols);

    // Skip label 0. This is the background, i.e. non-planar region.
    for (int label = 1; label <= number_of_extracted_planes_without_refinement; ++label) {
        computePlaneParametersForLabel(label, m_points_with_normal);
    }
}

void PlanarImageSegmentation::computePlaneParametersForLabel(
                    int label, std::vector<ransac_segmentation::PointWithNormal>& points_with_normal)
{
    const auto& elevationData = (*m_map)[m_elevation_layer];
    points_with_normal.clear(); 

    int num_points = 0;
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    Eigen::Matrix3d sum_squared = Eigen::Matrix3d::Zero();
    for (int col = 0; col < m_segmented_planes_map.labeled_image.cols; ++col) {
        for (int row = 0; row < m_segmented_planes_map.labeled_image.rows; ++row) {
            if (m_segmented_planes_map.labeled_image.at<int>(row, col) == label) {
                double height = elevationData(row, col);
                if (std::isfinite(height)) {
                    const Eigen::Vector3d point3d { m_segmented_planes_map.map_origin.x()
                            - row * m_segmented_planes_map.resolution,
                        m_segmented_planes_map.map_origin.y() - col * m_segmented_planes_map.resolution, height };

                    ++num_points;
                    sum += point3d;
                    sum_squared.noalias() += point3d * point3d.transpose();

                    const auto& local_surface_normal = m_surface_normals[getLinearIndex(row, col)];
                    points_with_normal.emplace_back(ransac_segmentation::Point3D(point3d.x(), point3d.y(), point3d.z()),
                        ransac_segmentation::Vector3D(
                            local_surface_normal.x(), local_surface_normal.y(), local_surface_normal.z()));
                }
            }
        }
    }
    if (num_points < m_parameters.min_number_points_per_label || num_points < 3) {
        // Label has too little points, no plane parameters are created
        return;
    }

    const Eigen::Vector3d support_vector = sum / num_points;
    const Eigen::Vector3d normal_vector = normalAndErrorFromCovariance(num_points, support_vector, sum_squared).first;

    if (m_parameters.include_ransac_refinement) {
        if (isGloballyPlanar(normal_vector, support_vector, points_with_normal)) { // Already planar enough
            if (isWithinInclinationLimit(normal_vector)) {
                m_segmented_planes_map.label_plane_parameters.emplace_back(
                    label, NormalAndPosition { support_vector, normal_vector });
            } else {
                setToBackground(label);
            }
        } else {
            // Set entire label to background, so unassigned points are automatically in background
            setToBackground(label);
            refineLabelWithRansac(label, points_with_normal);
        }
    } else { // no RANSAC
        if (isWithinInclinationLimit(normal_vector)) {
            m_segmented_planes_map.label_plane_parameters.emplace_back(
                label, NormalAndPosition { support_vector, normal_vector });
        } else {
            setToBackground(label);
        }
    }
}

void PlanarImageSegmentation::refineLabelWithRansac(
    int label, std::vector<ransac_plane_extractor::PointWithNormal>& points_with_normal) {

    CGAL::get_default_random() = CGAL::Random(0);

    ransac_segmentation::RansacSegmentation ransac_segmentation(m_ransac_parameters);
    ransac_segmentation.detectPlanes(points_with_normal);
    const auto& planes = ransac_plane_extractor.getDetectedPlanes();

    bool reuse_label = true;
    for (const auto& plane : planes) {
        const auto plane_info = ransac_plane_extractor::RansacPlaneExtractor::getPlaneParameters(plane.get());
        const auto& normal_vector = plane_info.first;
        const auto& support_vector = plane_info.second;

        if (isWithinInclinationLimit(normal_vector)) {
            // reuse old label for the first plane
            const int new_label = (reuse_label) ? label : ++m_segmented_planes_map.highestLabel;
            reuse_label = false;

            m_segmented_planes_map.label_plane_parameters.emplace_back(
                new_label, NormalAndPosition { support_vector, normal_vector });

            // Assign label in segmentation
            for (const auto index : plane->indices_of_assigned_points()) {
                const auto& point = points_with_normal[index].first;

                // Need to lookup indices in map, because RANSAC has reordered the points
                Eigen::Array2i map_indices;
                m_map->getIndex(Eigen::Vector2d(point.x(), point.y()), map_indices);
                m_segmented_planes_map.labeled_image.at<int>(map_indices(0), map_indices(1)) = new_label;
            }
        }
    }
}

void PlanarImageSegmentation::addSurfaceNormalToMap(grid_map::GridMap& map, const std::string& layer_prefix) const
{
    map.add(layer_prefix + "_x");
    map.add(layer_prefix + "_y");
    map.add(layer_prefix + "_z");
    auto& data_x = map.get(layer_prefix + "_x");
    auto& data_y = map.get(layer_prefix + "_y");
    auto& data_z = map.get(layer_prefix + "_z");

    grid_map::SlidingWindowIterator window_iterator(
        map, layer_prefix + "_x", grid_map::SlidingWindowIterator::EdgeHandling::EMPTY, m_parameters.kernel_size);

    for (; !window_iterator.isPastEnd(); ++window_iterator) {
        grid_map::Index index = *window_iterator;
        const auto& n = m_surface_normals[getLinearIndex(index.x(), index.y())];
        data_x(index.x(), index.y()) = n.x();
        data_y(index.x(), index.y()) = n.y();
        data_z(index.x(), index.y()) = n.z();
    }
}

bool PlanarImageSegmentation::isGloballyPlanar(const Eigen::Vector3d& normal_vector_plane,
    const Eigen::Vector3d& support_vector_plane,
    const std::vector<ransac_segmentation::PointWithNormal>& points_with_normal) const
{
    // Part of the plane projection that is independent of the point
    const double normal_dot_support_vector = normal_vector_plane.dot(support_vector_plane);

    // Convert threshold in degrees to threshold on dot product (between normalized vectors)
    const double dot_product_threshold
        = std::cos(m_parameters.global_plane_fit_angle_error_threshold_degrees * M_PI / 180.0);

    for (const auto& point_with_normal : points_with_normal) {

        const double normal_dot_point = normal_vector_plane.x() * point_with_normal.first.x()
            + normal_vector_plane.y() * point_with_normal.first.y()
            + normal_vector_plane.z() * point_with_normal.first.z();
        const double distance_error = std::abs(normal_dot_point - normal_dot_support_vector);
        const double dot_product_normals = normal_vector_plane.x() * point_with_normal.second.x()
            + normal_vector_plane.y() * point_with_normal.second.y()
            + normal_vector_plane.z() * point_with_normal.second.z();

        if (distance_error > m_parameters.global_plane_fit_distance_error_threshold
            || dot_product_normals < dot_product_threshold) {
            return false;
        }
    }

    return true;
}

bool PlanarImageSegmentation::isWithinInclinationLimit(const Eigen::Vector3d& normal_vector_plane) const
{
    return normal_vector_plane.z() > m_parameters.plane_inclination_threshold;
}

void PlanarImageSegmentation::setToBackground(int label)
{
    m_segmented_planes_map.labeled_image.setTo(0, m_segmented_planes_map.labeled_image == label);
}

} // namespace march_vision
