#include "utilities/transform_utilities.h"
//#include <pcl/point_types.h>

//using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

Transformer::Transformer(std::string fixed_frame = "world", std::string frame_id_to_transform_to)
{
    tfBuffer = std::make_unique<tf2_ros::Buffer>();
    tfListener = std::make_unique<tf2_ros::TransformListener>(*tfBuffer);

    fixed_frame_ = fixed_frame;
    frame_id_to_transform_to_ = frame_id_to_transform_to;
}

void Transformer::createTransform()
{
    if (tfBuffer->canTransform(
        fixed_frame_, frame_id_to_transform_to_, ros::Time(/*t=*/0), &error_str))
    {
    transform_stamped_msg = tfBuffer->lookupTransform(
        fixed_frame_, frame_id_to_transform_to_, ros::Time(/*t=*/0));
    } else {
        ROS_WARN_STREAM(error_str);
    }
    setYawQuaternion();
}

void Transformer::setYawQuaternion()
{
    tf2::convert(transform_stamped_msg.transform.rotation, transform_quaternion);
    transform_matrix.setRotation(transform_quaternion);
    transform_matrix.getEulerYPR(yaw, pitch, roll);
    yaw_quaternion.setRPY(/*roll=*/0, /*pitch=*/0, yaw);
}
