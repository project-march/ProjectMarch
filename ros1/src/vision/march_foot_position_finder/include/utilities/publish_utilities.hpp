/**
 * @author Tuhin Das - MARCH 7
 */

#ifndef MARCH_PUBLISH_UTILITIES
#define MARCH_PUBLISH_UTILITIES

#include <librealsense2/rs.hpp>
#include <march_shared_msgs/FootPosition.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

std::string world_frame = "world";

/**
 * Converts a PCL point to a ROS Point message.
 *
 * @param p point to convert
 * @return converted geometry_msgs::Point message
 */
inline geometry_msgs::Point to_geometry(Point p)
{
    geometry_msgs::Point msg;
    msg.x = p.x;
    msg.y = p.y;
    msg.z = p.z;
    return msg;
}

/**
 * Transforms a pointcloud to world frame and publishes it for visualization. A
 * rotation around the z-axis is necessary to align the pointcloud with the
 * world coordinate system.
 *
 * @param publisher publisher to use
 * @param cloud cloud to publish
 */
void publishCloud(const ros::Publisher& publisher, PointCloud cloud)
{
    cloud.width = 1;
    cloud.height = cloud.points.size();
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = world_frame;
    msg.header.stamp = ros::Time::now();
    publisher.publish(msg);
}

/**
 * Publishes a marker point with a given publisher. A rotation around the z
 * axis is needed to correctly align the realsense and world coordinate systems.
 *
 * @param publisher publisher to use
 * @param p point to publish
 */
void publishMarkerPoint(ros::Publisher& publisher, const Point& p)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = world_frame;
    marker.header.stamp = ros::Time::now();

    marker.ns = "found_points";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

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
    marker.lifetime = ros::Duration(/*t=*/0.3);

    publisher.publish(marker);
}

void publishArrow(ros::Publisher& publisher, const Point& p1, Point& p2)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = world_frame;
    marker.header.stamp = ros::Time::now();

    marker.ns = "displacement";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

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
    marker.lifetime = ros::Duration(/*t=*/0.3);

    marker.points.push_back(to_geometry(p1));
    marker.points.push_back(to_geometry(p2));

    publisher.publish(marker);
}

void publishArrow2(ros::Publisher& publisher, const Point& p1, Point& p2)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = world_frame;
    marker.header.stamp = ros::Time::now();

    marker.ns = "displacement_computed";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

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
    marker.lifetime = ros::Duration(/*t=*/0.3);

    marker.points.push_back(to_geometry(p1));
    marker.points.push_back(to_geometry(p2));

    publisher.publish(marker);
}

/**
 * Publishes a marker point with a given publisher. A rotation around the z
 * axis is needed to correctly align the realsense and world coordinate systems.
 *
 * @param publisher publisher to use
 * @param p point to publish
 */
void publishRelativeSearchPoint(ros::Publisher& publisher, const Point& p)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = world_frame;
    marker.header.stamp = ros::Time::now();

    marker.ns = "relative_points";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

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
    marker.lifetime = ros::Duration(/*t=*/0.3);

    publisher.publish(marker);
}

/**
 * Publishes a marker point with a given publisher. A rotation around the z
 * axis is needed to correctly align the realsense and world coordinate systems.
 *
 * @param publisher publisher to use
 * @param p point to publish
 */
void publishDesiredPosition(ros::Publisher& publisher, const Point& p)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = world_frame;
    marker.header.stamp = ros::Time::now();

    marker.ns = "desired_position";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

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
    marker.lifetime = ros::Duration(/*t=*/0.3);

    publisher.publish(marker);
}

/**
 * Publishes a rectangle with a given publisher. A rotation around the z
 * axis is needed to correctly align the realsense and world coordinate systems.
 *
 * @param publisher publisher to use
 * @param p1 vertex of rectangle
 * @param p2 vertex of rectangle
 * @param p3 vertex of rectangle
 * @param p4 vertex of rectangle
 */
void publishSearchRectangle(ros::Publisher& publisher, Point& p,
    std::vector<double> dis, const std::string& left_or_right)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = world_frame;
    marker.header.stamp = ros::Time::now();

    marker.ns = "search_region";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

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
    marker.lifetime = ros::Duration(/*t=*/0.3);

    publisher.publish(marker);
}

/**
 * Publishes a list of points to visualize.
 *
 * @param publisher publisher to use
 * @param points points to visualize
 */
void publishPossiblePoints(
    ros::Publisher& publisher, std::vector<Point>& points)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = world_frame;
    marker.header.stamp = ros::Time::now();

    marker.ns = "possible_points";
    marker.id = 2;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    for (Point& point : points) {
        marker.points.push_back(to_geometry(point));
    }

    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.0055;
    marker.scale.y = 0.0055;

    marker.color.g = 0.75;
    marker.color.b = 0.25;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(/*t=*/0.2);

    publisher.publish(marker);
}

/**
 * Publishes a list of track points to visualize.
 *
 * @param publisher publisher to use
 * @param points points to visualize
 */
void publishTrackMarkerPoints(
    ros::Publisher& publisher, std::vector<Point>& points)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = world_frame;
    marker.header.stamp = ros::Time::now();

    marker.ns = "track_points";
    marker.id = 2;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    for (auto& point : points) {
        marker.points.push_back(to_geometry(point));
    }

    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.0055;
    marker.scale.y = 0.0055;

    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(/*t=*/0.2);

    publisher.publish(marker);
}

/**
 * Publishes a point to a given publisher.
 *
 * @param publisher publisher to use
 * @param p point to publish
 * @param track_points vector of points between start and end position of this
 * step
 */
void publishPoint(ros::Publisher& publisher, Point& p, Point& p_world,
    Point& displacement, const std::vector<Point>& track_points)
{
    march_shared_msgs::FootPosition msg;

    msg.point.x = p.x;
    msg.point.y = p.y;
    msg.point.z = p.z;

    msg.point_world.x = p_world.x;
    msg.point_world.y = p_world.y;
    msg.point_world.z = p_world.z;

    msg.displacement.x = displacement.x;
    msg.displacement.y = displacement.y;
    msg.displacement.z = displacement.z;

    msg.header.stamp = ros::Time::now();

    for (const Point& p : track_points) {
        msg.track_points.push_back(to_geometry(p));
    }

    publisher.publish(msg);
}

#endif // MARCH_PUBLISH_UTILITIES
