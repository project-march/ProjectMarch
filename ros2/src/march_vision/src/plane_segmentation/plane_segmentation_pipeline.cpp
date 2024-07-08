// #include "plane_segmentation/plane_segmentation_pipeline.h"

// #include <opencv2/core/eigen.hpp>

// namespace plane_segmentation {

// PlaneSegmentationPipeline::PlaneSegmentationPipeline(std::shared_ptr<rclcpp::Node>& node_handle)
//     : m_node_handle(node_handle)
//       {
//         RCLCPP_INFO(m_node_handle->get_logger(), "Plane segmentation node started.");
//         bool is_parameters_read = readParameters();
//         if(is_parameters_read){

//           // Input: filtered/postprocessed elevation map
//           m_elevation_map_sub = nodeHandle.subscribe("elevation_map_filtered", 1, &PlaneSegmentationPipeline::callback, this);
//           // RViz visualizations
//           m_boundary_pub = nodeHandle.advertise<visualization_msgs::MarkerArray>("boundaries", 1);
//           m_inset_pub = nodeHandle.advertise<visualization_msgs::MarkerArray>("insets", 1);

//           // TODO: Do I need filteredmapPublisher_?
//           // Output: planar terrain
//           m_region_pub = nodeHandle.advertise<convex_plane_decomposition_msgs::PlanarTerrain>("planar_terrain", 1);
//         }
//         // m_tf_buffer = std::make_shared<tf2_ros::Buffer>(m_node_handle->get_clock());
//         // m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
//       }

// // TODO: Add timers?
// void PlaneSegmentationPipeline::update(grid_map::GridMap&& grid_map, const std::string& elevation_layer) {
//   // Clear/overwrite old result
//   m_planar_terrain.planar_regions.clear();
//   m_planar_terrain.grid_map = std::move(grid_map);
 
//   m_planar_image_segmentation.runExtraction(m_planar_terrain.grid_map, elevation_layer);
//   m_planar_terrain.planar_regions = m_contour_segmentation.extractPlanarRegions(m_planar_image_segmentation.getSegmentedPlanesMap());
//   // TODO: Don't know if we're going to use this
//   //m_postprocessing.postprocess(m_planar_terrain, elevation_layer, planeClassificationLayer);
// }

// void PlaneSegmentationPipeline::getSegmentation(grid_map::GridMap::Matrix& segmentation) const {
//   cv::cv2eigen(m_planar_image_segmentation.getSegmentedPlanesMap().labeled_image, segmentation);
// }

// // TODO: Make sure it reads the inheritece correctly and include defaults here maybe
// bool PlaneSegmentationPipeline::readParameters(){

//   m_config = std::make_shared<Config>();

//   m_node_handle->declare_parameter("planar_image_segmentation.kernel_size");
//   m_node_handle->declare_parameter("planar_image_segmentation.planar_opening_filter");
//   m_node_handle->declare_parameter("planar_image_segmentation.plane_inclination_threshold_degrees");
//   m_node_handle->declare_parameter("planar_image_segmentation.local_plane_inclination_threshold_degrees");
//   m_node_handle->declare_parameter("planar_image_segmentation.plane_patch_error_threshold");
//   m_node_handle->declare_parameter("planar_image_segmentation.min_number_points_per_label");
//   m_node_handle->declare_parameter("planar_image_segmentation.connectivity");
//   m_node_handle->declare_parameter("planar_image_segmentation.include_ransac_refinement");
//   m_node_handle->declare_parameter("planar_image_segmentation.global_plane_fit_distance_error_threshold");
//   m_node_handle->declare_parameter("planar_image_segmentation.global_plane_fit_angle_error_threshold_degrees");

//   m_node_handle->get_parameter("planar_image_segmentation.kernel_size", m_config.PlanarImageSegmentationParameters.kernel_size);
//   m_node_handle->get_parameter("planar_image_segmentation.planar_opening_filter", m_config.PlanarImageSegmentationParameters.planar_opening_filter);
//   m_node_handle->get_parameter("planar_image_segmentation.local_plane_inclination_threshold_degrees", m_config.PlanarImageSegmentationParameters.local_plane_inclination_threshold_degrees);
//   m_node_handle->get_parameter("planar_image_segmentation.plane_patch_error_threshold", m_config.PlanarImageSegmentationParameters.plane_patch_error_threshold);
//   m_node_handle->get_parameter("planar_image_segmentation.min_number_points_per_label", m_config.PlanarImageSegmentationParameters.min_number_points_per_label);
//   m_node_handle->get_parameter("planar_image_segmentation.connectivity", m_config.PlanarImageSegmentationParameters.connectivity);
//   m_node_handle->get_parameter("planar_image_segmentation.include_ransac_refinement", m_config.PlanarImageSegmentationParameters.include_ransac_refinement);
//   m_node_handle->get_parameter("planar_image_segmentation.global_plane_fit_distance_error_threshold", m_config.PlanarImageSegmentationParameters.global_plane_fit_distance_error_threshold);
//   m_node_handle->get_parameter("planar_image_segmentation.global_plane_fit_angle_error_threshold_degrees", m_config.PlanarImageSegmentationParameters.global_plane_fit_angle_error_threshold_degrees);

//   m_node_handle->declare_parameter("contour_segmentation.margin_size");

//   m_node_handle->get_parameter("contour_extraction.margin_size", m_config.ContourSegmentationParameters.margin_size);

//   m_node_handle->declare_parameter("ransac_segmentation.probability");
//   m_node_handle->declare_parameter("ransac_segmentation.min_points");
//   m_node_handle->declare_parameter("ransac_segmentation.epsilon");
//   m_node_handle->declare_parameter("ransac_segmentation.cluster_epsilon");
//   m_node_handle->declare_parameter("ransac_segmentation.normal_threshold");

//   m_node_handle->get_parameter("ransac_segmentation.probability", m_config.RansacSegmentationParameters.probability);
//   m_node_handle->get_parameter("ransac_segmentation.min_points", m_config.RansacSegmentationParameters.min_points);
//   m_node_handle->get_parameter("ransac_segmentation.epsilon", m_config.RansacSegmentationParameters.epsilon);
//   m_node_handle->get_parameter("ransac_segmentation.cluster_epsilon", m_config.RansacSegmentationParameters.cluster_epsilon);
//   m_node_handle->get_parameter("ransac_segmentation.normal_threshold", m_config.RansacSegmentationParameters.normal_threshold);

//   return true;
// }

// // TODO: Check if it should be grid_map_msgs::GridMap
// void PlaneSegmentationNode::planeSegmentationCallback(const grid_map::GridMap& message) {
//   //m_callback_timer.startTimer();

//   // Convert message to map.
//   grid_map::GridMap message_map;
//   // TODO: This should be a parameter from config "height_layer: 'elevation'" 
//   std::vector<std::string> layers{m_elevation_layer};
//   grid_map::GridMapRosConverter::fromMessage(message, message_map, layers, false, false);
//   if (!containsFiniteValue(message_map.get(m_elevation_layer))) {
//     ROS_WARN("[PlaneSegmentation] map does not contain any values");
//     callbackTimer_.endTimer();
//     return;
//   }

//   const grid_map::Matrix elevation_raw = elevationMap.get(m_elevation_layer);

//   update(std::move(elevationMap), m_elevation_layer);
//   auto& planar_terrain = m_planar_terrain;

//   m_region_publisher.publish(toMessage(planar_terrain));

//   // --- Visualize in Rviz ---
//   // Not published to the controller
//   planar_terrain.gridMap.add("elevation_raw", elevation_raw);
//   planar_terrain.gridMap.add("segmentation");
//   getSegmentation(planar_terrain.gridMap.get("segmentation"));

//   grid_map_msgs::GridMap output_msg;
//   grid_map::GridMapRosConverter::toMessage(planar_terrain.gridMap, output_msg);
//   filteredmapPublisher_.publish(output_msg);

//   const double line_width = 0.005;  // [m] RViz marker size
//   m_boundary_pub.publish(convertBoundariesToRosMarkers(planar_terrain.planar_regions, planar_terrain.grid_map.getFrameId(),
//                                                        planar_terrain.grid_map.getTimestamp(), line_width));
//   m_inset_pub.publish(convertInsetsToRosMarkers(planar_terrain.planar_regions, planar_terrain.grid_map.getFrameId(),
//                                                 planar_terrain.grid_map.getTimestamp(), line_width));

//   //m_callback_timer.endTimer();
// }


// }  // namespace plane_segmentation