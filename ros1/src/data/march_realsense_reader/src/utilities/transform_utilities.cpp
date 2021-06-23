#include "utilities/transform_utilities.h"
#include <pcl_ros/transforms.h>
//#include <pcl/point_types.h>

// using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

Transformer::Transformer(
    std::string frame_id_to_transform_to, std::string fixed_frame)
{
    tfBuffer = std::make_unique<tf2_ros::Buffer>();
    tfListener = std::make_unique<tf2_ros::TransformListener>(*tfBuffer);

    fixed_frame_ = fixed_frame;
    frame_id_to_transform_to_ = frame_id_to_transform_to;
    createTransform();
    copyYawToTransform();
    ROS_DEBUG("Created Transformer");
}

void Transformer::transformPointCloud(const PointCloud::Ptr& cloud)
{
    pcl_ros::transformPointCloud(
        *cloud, *cloud, transform_stamped_msg.transform);
}

void Transformer::createTransform()
{
    if (tfBuffer->canTransform(fixed_frame_, frame_id_to_transform_to_,
            ros::Time(/*t=*/0), ros::Duration(1.0), &error_str)) {
        transform_stamped_msg = tfBuffer->lookupTransform(
            fixed_frame_, frame_id_to_transform_to_, ros::Time(/*t=*/0));
        ROS_DEBUG_STREAM(
            "Can transform. frame_id: " << frame_id_to_transform_to_);
    } else {
        ROS_WARN_STREAM(error_str);
    }
    setYaw();
    ROS_DEBUG("Created Transform");
}

void Transformer::setYaw()
{
    tf2::convert(
        transform_stamped_msg.transform.rotation, transform_quaternion);
    transform_matrix.setRotation(transform_quaternion);
    transform_matrix.getEulerYPR(yaw, pitch, roll);
    yaw_quaternion.setRPY(/*roll=*/0, /*pitch=*/0, yaw);
    tf2::convert(yaw_quaternion, yaw_stamped_msg.transform.rotation);
    ROS_DEBUG("yaw in yaw_quaternion is: %f", yaw);
}

void Transformer::copyYawToTransform()
{
    transform_stamped_msg.transform.rotation.x
        = yaw_stamped_msg.transform.rotation.x;
    transform_stamped_msg.transform.rotation.y
        = yaw_stamped_msg.transform.rotation.y;
    transform_stamped_msg.transform.rotation.z
        = yaw_stamped_msg.transform.rotation.z;
    transform_stamped_msg.transform.rotation.w
        = yaw_stamped_msg.transform.rotation.w;
}
