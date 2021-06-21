#include <tf2_ros/transform_listener.h>
#include <string>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/TransformStamped.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2/LinearMath/Matrix3x3.h>

#ifndef MARCH_TRANSFORM_UTILITIES_H
#define MARCH_TRANSFORM_UTILITIES_H

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

class Transformer {
public:
    explicit Transformer(std::string frame_id_to_transform_to, std::string fixed_frame="world");

    void transformPointCloud(const PointCloud::Ptr& cloud);
    void transformPoint(const pcl::PointXYZ point);

private:
    void createTransform();
    void setYawQuaternion();

    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tfListener;

    geometry_msgs::TransformStamped transform_stamped_msg, yaw_stamped_msg;
    tf2::Quaternion transform_quaternion, yaw_quaternion;
    tf2::Matrix3x3 transform_matrix;
    tf2Scalar yaw, pitch, roll;

    std::string fixed_frame_;
    std::string frame_id_to_transform_to_;

    std::string error_str;

};
#endif // MARCH_TRANSFORM_UTIlITIES_H
