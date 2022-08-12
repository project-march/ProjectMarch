/**
 * @author Tuhin Das - MARCH 7
 */

#ifndef MARCH_PUBLISH_UTILITIES
#define MARCH_PUBLISH_UTILITIES

#include "march_shared_msgs/msg/foot_position.hpp"
#include <librealsense2/rs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace marchPublishUtilities {

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudPublisher = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>;
using MarkerPublisher = rclcpp::Publisher<visualization_msgs::msg::Marker>;

std::string world_frame = "world";

/**
 * Converts a PCL point to a ROS Point message.
 *
 * @param p point to convert
 * @return converted geometry_msgs::msg::Point message
 */
inline geometry_msgs::msg::Point to_geometry(Point p)
{
    geometry_msgs::msg::Point msg;
    msg.x = p.x;
    msg.y = p.y;
    msg.z = p.z;
    return msg;
}

/**
 * Transforms a pointcloud to world frame and publishes it for visualization.
 *
 * @param publisher publisher to use
 * @param cloud cloud to publish
 */
inline void publishCloud(
    const PointCloudPublisher::SharedPtr& publisher, rclcpp::Node* n, PointCloud cloud, std::string& left_or_right)
{
    cloud.width = 1;
    cloud.height = cloud.points.size();
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    if (left_or_right == "right") {
        msg.header.frame_id = "toes_right_aligned";
    } else {
        msg.header.frame_id = "toes_left_aligned";
    }
    msg.header.stamp = n->now();
    publisher->publish(msg);
}

/**
 * Publishes a marker point with a given publisher->
 *
 * @param publisher publisher to use
 * @param p point to publish
 */
inline void publishMarkerPoint(
    const MarkerPublisher::SharedPtr& publisher, rclcpp::Node* n, const Point& p, std::string& left_or_right)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "toes_" + left_or_right + "_aligned";
    marker.header.stamp = n->now();

    marker.ns = "found_points";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = p.x;
    marker.pose.position.y = p.y;
    marker.pose.position.z = p.z;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(/*seconds=*/0.3);

    publisher->publish(marker);
}

inline void publishPreviousDisplacement(const MarkerPublisher::SharedPtr& publisher, rclcpp::Node* n, const Point& p1,
    Point& p2, std::string& left_or_right)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "toes_" + left_or_right + "_aligned";
    marker.header.stamp = n->now();

    marker.ns = "displacement";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.02;
    marker.scale.y = 0.03;

    marker.color.r = 0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(/*seconds=*/0.3);

    marker.points.push_back(to_geometry(p1));
    marker.points.push_back(to_geometry(p2));

    publisher->publish(marker);
}

inline void publishNewDisplacement(const MarkerPublisher::SharedPtr& publisher, rclcpp::Node* n, const Point& p1,
    Point& p2, std::string& left_or_right)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "toes_" + left_or_right + "_aligned";
    marker.header.stamp = n->now();

    marker.ns = "displacement_computed";
    marker.id = 2;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.02F;
    marker.scale.y = 0.03F;

    marker.color.r = 0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(/*seconds=*/0.3);

    marker.points.push_back(to_geometry(p1));
    marker.points.push_back(to_geometry(p2));

    publisher->publish(marker);
}

/**
 * Publishes a marker point with a given publisher->
 *
 * @param publisher publisher to use
 * @param p point to publish
 */
inline void publishRelativeSearchPoint(
    const MarkerPublisher::SharedPtr& publisher, rclcpp::Node* n, const Point& p, std::string& left_or_right)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "toes_" + left_or_right + "_aligned";
    marker.header.stamp = n->now();

    marker.ns = "relative_points";
    marker.id = 3;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = p.x;
    marker.pose.position.y = p.y;
    marker.pose.position.z = p.z;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(/*seconds=*/0.3);

    publisher->publish(marker);
}

/**
 * Publishes a marker point with a given publisher->
 *
 * @param publisher publisher to use
 * @param p point to publish
 */
inline void publishDesiredPosition(
    const MarkerPublisher::SharedPtr& publisher, rclcpp::Node* n, const Point& p, std::string& left_or_right)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "toes_" + left_or_right + "_aligned";
    marker.header.stamp = n->now();

    marker.ns = "desired_position";
    marker.id = 4;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = p.x;
    marker.pose.position.y = p.y;
    marker.pose.position.z = p.z;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(/*seconds=*/0.3);

    publisher->publish(marker); // NOLINT
}

/**
 * Publishes a rectangle with a given publisher->
 *
 * @param publisher publisher to use
 * @param p1 vertex of rectangle
 * @param p2 vertex of rectangle
 * @param p3 vertex of rectangle
 * @param p4 vertex of rectangle
 */
inline void publishSearchRectangle(const MarkerPublisher::SharedPtr& publisher, rclcpp::Node* n, Point& p,
    std::vector<double> dis, const std::string& left_or_right)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "toes_" + left_or_right + "_aligned";
    marker.header.stamp = n->now();

    marker.ns = "search_region";
    marker.id = 5;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    float outside;
    float inside;

    if (left_or_right == "right") {
        outside = dis[1];
        inside = dis[0];
    } else {
        outside = dis[0];
        inside = dis[1];
    }

    Point p1((float)(p.x - dis[3]), (float)(p.y + inside), /*_z=*/0);
    Point p2((float)(p.x - dis[3]), (float)(p.y - outside), /*_z=*/0);
    Point p3((float)(p.x + dis[2]), (float)(p.y - outside), /*_z=*/0);
    Point p4((float)(p.x + dis[2]), (float)(p.y + inside), /*_z=*/0);

    marker.points.push_back(to_geometry(p1));
    marker.points.push_back(to_geometry(p2));
    marker.points.push_back(to_geometry(p3));
    marker.points.push_back(to_geometry(p4));
    marker.points.push_back(to_geometry(p1));

    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.01;

    marker.color.g = 1.0;
    marker.color.b = 0.8;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(/*seconds=*/0.3);

    publisher->publish(marker);
}

/**
 * Publishes a rectangle around the foot point.
 *
 */
inline void publishFootRectangle(
    const MarkerPublisher::SharedPtr& publisher, rclcpp::Node* n, Point& p, const std::string& left_or_right)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "toes_" + left_or_right + "_aligned";
    marker.header.stamp = n->now();

    marker.ns = "foot_rectangle";
    marker.id = 9;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    Point p1((float)(p.x), (float)(p.y + 0.05), /*_z=*/(float)p.z);
    Point p2((float)(p.x), (float)(p.y - 0.05), /*_z=*/(float)p.z);
    Point p3((float)(p.x + 0.20), (float)(p.y - 0.05), /*_z=*/(float)p.z);
    Point p4((float)(p.x + 0.20), (float)(p.y + 0.05), /*_z=*/(float)p.z);

    marker.points.push_back(to_geometry(p1));
    marker.points.push_back(to_geometry(p2));
    marker.points.push_back(to_geometry(p3));
    marker.points.push_back(to_geometry(p4));
    marker.points.push_back(to_geometry(p1));

    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.01;

    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(/*seconds=*/0.3);

    publisher->publish(marker);
}

/**
 * Publishes a list of points to visualize.
 *
 * @param publisher publisher to use
 * @param points points to visualize
 */
inline void publishPossiblePoints(const MarkerPublisher::SharedPtr& publisher, rclcpp::Node* n,
    std::vector<Point>& points, std::string& left_or_right)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "toes_" + left_or_right + "_aligned";
    marker.header.stamp = n->now();

    marker.ns = "possible_points";
    marker.id = 6;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;

    for (Point& point : points) {
        marker.points.push_back(to_geometry(point));
    }

    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.0055;
    marker.scale.y = 0.0055;

    marker.color.g = 0.75;
    marker.color.b = 0.25;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(/*seconds=*/0.3);

    publisher->publish(marker);
}

/**
 * Publishes a list of track points to visualize.
 *
 * @param publisher publisher to use
 * @param points points to visualize
 */
inline void publishTrackMarkerPoints(const MarkerPublisher::SharedPtr& publisher, rclcpp::Node* n,
    std::vector<Point>& points, std::string& left_or_right)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "toes_" + left_or_right + "_aligned";
    marker.header.stamp = n->now();

    marker.ns = "track_points";
    marker.id = 7;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;

    for (auto& point : points) {
        marker.points.push_back(to_geometry(point));
    }

    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.0055;
    marker.scale.y = 0.0055;

    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(/*seconds=*/0.3);

    publisher->publish(marker);
}

/**
 * Publishes a marker point with a given publisher
 *
 * @param publisher publisher to use
 * @param p point to publish
 */
inline void publishOriginalMarkerPoint(
    const MarkerPublisher::SharedPtr& publisher, rclcpp::Node* n, const Point& p, std::string& left_or_right)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "toes_" + left_or_right + "_aligned";
    marker.header.stamp = n->now();

    marker.ns = "original_point";
    marker.id = 8;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = p.x;
    marker.pose.position.y = p.y;
    marker.pose.position.z = p.z;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;

    marker.color.r = 0.1;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(/*seconds=*/0.3);

    publisher->publish(marker); // NOLINT
}

/**
 * Publishes a point to a given publisher->
 *
 * @param publisher publisher to use
 * @param p point to publish
 * @param track_points vector of points between start and end position of this
 * step
 */
inline void publishPoint(const rclcpp::Publisher<march_shared_msgs::msg::FootPosition>::SharedPtr& publisher,
    rclcpp::Node* n, Point& p, Point& p_world, Point& displacement, const std::vector<Point>& track_points)
{
    march_shared_msgs::msg::FootPosition msg;

    msg.point.x = p.x;
    msg.point.y = p.y;
    msg.point.z = p.z;

    msg.point_world.x = p_world.x;
    msg.point_world.y = p_world.y;
    msg.point_world.z = p_world.z;

    msg.displacement.x = displacement.x;
    msg.displacement.y = displacement.y;
    msg.displacement.z = displacement.z;

    msg.header.stamp = n->now();

    for (const Point& p : track_points) {
        msg.track_points.push_back(to_geometry(p));
    }

    publisher->publish(msg);
}

} // namespace marchPublishUtilities

#endif // MARCH_PUBLISH_UTILITIES
