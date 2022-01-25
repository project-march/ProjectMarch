#ifndef MARCH_PUBLISH_UTILITIES
#define MARCH_PUBLISH_UTILITIES

#include <librealsense2/rs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

/**
 * Publishes a point cloud with a given publisher.
 *
 * @param publisher publisher to use
 * @param cloud cloud to publish
 */
void publishCloud(const ros::Publisher& publisher, PointCloud cloud)
{
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    // rotate point around z axis (counter clockwise)
    transform.rotate(Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(cloud, cloud, transform);

    cloud.width = 1;
    cloud.height = cloud.points.size();

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();
    publisher.publish(msg);
}

/**
 * Publishes a marker point with a given publisher.
 *
 * @param publisher publisher to use
 * @param p point to publish
 */
void publishMarkerPoint(ros::Publisher& publisher, Point& p)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();

    marker.ns = "found_points";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    // rotate point around z axis (counter clockwise)
    marker.pose.position.x = -p.y;
    marker.pose.position.y = p.x;
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
    marker.lifetime = ros::Duration(0.3);

    publisher.publish(marker);
}

/**
 * Publishes a point to a given publisher.
 *
 * @param publisher publisher to use
 * @param p point to publish
 */
void publishPoint(ros::Publisher& publisher, Point& p)
{
    geometry_msgs::PointStamped msg;
    msg.point.x = p.x;
    msg.point.y = p.y;
    msg.point.z = p.z;
    msg.header.stamp = ros::Time::now();

    publisher.publish(msg);
}

#endif // MARCH_PUBLISH_UTILITIES
