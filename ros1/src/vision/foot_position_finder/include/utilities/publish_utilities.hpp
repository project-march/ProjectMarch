#pragma once

#include <librealsense2/rs.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/MarkerArray.h>

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;


void publishCloud(const ros::Publisher& publisher, PointCloud cloud)
{

    ROS_INFO("Publish cloud");


    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(cloud, cloud, transform);

    cloud.width = 1;
    cloud.height = cloud.points.size();

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();
    publisher.publish(msg);
}


void publishMarkerPoint(const ros::Publisher& publisher, Point &p)
{

    ROS_INFO("Publish marker");

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

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();

    publisher.publish(marker);
}
  