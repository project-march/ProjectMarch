#include "utilities/transform_utilities.h"
#include <pcl_ros/transforms.h>

Transformer::Transformer(
    std::string frame_id_to_transform_to, std::string fixed_frame)
{
    tfBuffer = std::make_unique<tf2_ros::Buffer>();
    tfListener = std::make_unique<tf2_ros::TransformListener>(*tfBuffer);
    yaw = 0.0;
    pitch = 0.0;
    roll = 0.0;

    fixed_frame_ = std::move(fixed_frame);
    frame_id_to_transform_to_ = std::move(frame_id_to_transform_to);
    createTransform();
    copyYawToTransform();
}

void Transformer::transformPointCloud(const PointCloud::Ptr& cloud)
{
    pcl_ros::transformPointCloud(
        *cloud, *cloud, transform_stamped_msg.transform);
}

void Transformer::transformPoint(std::shared_ptr<pcl::PointXYZ>& point)
{
    boost::shared_ptr<PointCloud> cloud = boost::make_shared<PointCloud>();
    cloud->push_back(*point);
    transformPointCloud(cloud);
    point->x = cloud->at(/*n=*/0).x;
    point->y = cloud->at(/*n=*/0).y;
    point->z = cloud->at(/*n=*/0).z;
}

std::string Transformer::getFixedFrame()
{
    return fixed_frame_;
}

void Transformer::createTransform()
{
    if (tfBuffer->canTransform(fixed_frame_, frame_id_to_transform_to_,
            ros::Time(), ros::Duration(/*t=*/1.0), &error_str)) {
        transform_stamped_msg = tfBuffer->lookupTransform(
            fixed_frame_, frame_id_to_transform_to_, ros::Time(/*t=*/0));
        ROS_DEBUG_STREAM(
            "Can transform. frame_id: " << frame_id_to_transform_to_);
    } else {
        transform_stamped_msg.transform.translation.x = 0;
        transform_stamped_msg.transform.translation.y = 0;
        transform_stamped_msg.transform.translation.z = 0;
        transform_stamped_msg.transform.rotation.x = 0;
        transform_stamped_msg.transform.rotation.y = 0;
        transform_stamped_msg.transform.rotation.z = 0;
        transform_stamped_msg.transform.rotation.w = 1;
        ROS_WARN_STREAM(error_str);
    }
    setYaw();
}

void Transformer::setYaw()
{
    tf2::convert(
        transform_stamped_msg.transform.rotation, transform_quaternion);
    transform_matrix.setRotation(transform_quaternion);
    transform_matrix.getEulerYPR(yaw, pitch, roll);
    yaw_quaternion.setRPY(/*roll=*/0, /*pitch=*/0, yaw);
    tf2::convert(yaw_quaternion, yaw_stamped_msg.transform.rotation);
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
