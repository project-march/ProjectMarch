#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <math.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

using TransformStamped = geometry_msgs::msg::TransformStamped;

class FramePublisherNode : public rclcpp::Node {
public:
    FramePublisherNode()
        : Node("march_frame_publisher")
    {

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_
            = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_
            = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), [this]() -> void {
                publishToeFrames();
                publishCameraFrame("left");
                publishCameraFrame("right");
            });

        callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&FramePublisherNode::parametersCallback, this,
                std::placeholders::_1));

        range.set__from_value(-10.0).set__to_value(10.0).set__step(0.1);
        descriptor.floating_point_range = { range };
        this->declare_parameter<double>(
            "rotation_camera_left", rotation_camera_left, descriptor);
        this->declare_parameter<double>(
            "rotation_camera_right", rotation_camera_right, descriptor);
    }

private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_ { nullptr };
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr publish_timer_;

    std::string robot_properties_path
        = ament_index_cpp::get_package_share_directory("march_description")
        + "/urdf/properties/march7.yaml";
    YAML::Node robot_properties = YAML::LoadFile(robot_properties_path);
    const double TRANS_X
        = -robot_properties["dimensions"]["foot"]["height_forward"]
               .as<double>();
    const double TRANS_Z = -0.025;

    TransformStamped trans_left_;
    TransformStamped trans_right_;
    rclcpp::Time last_published_left_;
    rclcpp::Time last_published_right_;
    bool start_time_initialized = false;

    double rotation_camera_left = 0.0;
    double rotation_camera_right = 0.0;

    TransformStamped tr;
    rclcpp::Time last_published_camera_left = tr.header.stamp;
    rclcpp::Time last_published_camera_right = tr.header.stamp;

    rcl_interfaces::msg::ParameterDescriptor descriptor;
    rcl_interfaces::msg::FloatingPointRange range;

    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter>& parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        for (const auto& param : parameters) {
            if (param.get_name() == "rotation_camera_left") {
                rotation_camera_left = param.as_double();
            }
            if (param.get_name() == "rotation_camera_right") {
                rotation_camera_right = param.as_double();
            }
            parameterUpdatedLogger(param);
        }
        return result;
    }

    void parameterUpdatedLogger(rclcpp::Parameter param)
    {
        RCLCPP_INFO(this->get_logger(),
            param.get_name() + " set to " + param.value_to_string());
    }

    void publishCameraFrame(std::string left_or_right)
    {
        tr.header.stamp = this->get_clock()->now();
        if ((left_or_right == "left"
                && rclcpp::Time(tr.header.stamp) > last_published_camera_left)
            || (left_or_right == "right"
                && rclcpp::Time(tr.header.stamp)
                    > last_published_camera_right)) {
            tr.header.frame_id
                = "camera_front_" + left_or_right + "_depth_optical_frame";
            tr.child_frame_id
                = "camera_front_" + left_or_right + "_virtual_rotated";
            tf2::Quaternion tf2_quaternion;
            if (left_or_right == "left") {
                tf2_quaternion.setRPY(
                    rotation_camera_left / 180 * M_PI, 0.0, 0.0);
                last_published_camera_left = rclcpp::Time(tr.header.stamp);
            } else if (left_or_right == "right") {
                tf2_quaternion.setRPY(
                    rotation_camera_right / 180 * M_PI, 0.0, 0.0);
                last_published_camera_right = rclcpp::Time(tr.header.stamp);
            }

            tr.transform.rotation = tf2::toMsg(tf2_quaternion);
            tf_broadcaster_->sendTransform(tr);
        }
    }

    void publishToeFrames()
    {

        using namespace geometry_msgs::msg;

        try {
            trans_left_ = tf_buffer_->lookupTransform(
                "foot_left", "world", tf2::TimePointZero);
            trans_right_ = tf_buffer_->lookupTransform(
                "foot_right", "world", tf2::TimePointZero);

            if (!start_time_initialized) {
                last_published_left_ = trans_left_.header.stamp;
                last_published_right_ = trans_right_.header.stamp;
                start_time_initialized = true;
            }

        } catch (tf2::TransformException& ex) {
            return;
        }

        if (rclcpp::Time(trans_left_.header.stamp)
            > rclcpp::Time(last_published_left_)) {
            // Transformation from left foot to left toes
            TransformStamped tr1;
            tr1.header.stamp = trans_left_.header.stamp;
            tr1.header.frame_id = "foot_left";
            tr1.child_frame_id = "toes_left";
            tr1.transform.translation.x = TRANS_X;
            tr1.transform.translation.y = 0.0;
            tr1.transform.translation.z = TRANS_Z;
            tr1.transform.rotation = Quaternion();
            tf_broadcaster_->sendTransform(tr1);

            // Transformation from left toes to left aligned
            TransformStamped tr2;
            tr2.header.stamp = trans_left_.header.stamp;
            tr2.header.frame_id = "toes_left";
            tr2.child_frame_id = "toes_left_aligned";
            tr2.transform.rotation = trans_left_.transform.rotation;
            tf_broadcaster_->sendTransform(tr2);

            last_published_left_ = trans_left_.header.stamp;
        }

        if (rclcpp::Time(trans_right_.header.stamp)
            > rclcpp::Time(last_published_right_)) {
            // Transformation from right foot to right toes
            TransformStamped tr3;
            tr3.header.stamp = trans_right_.header.stamp;
            tr3.header.frame_id = "foot_right";
            tr3.child_frame_id = "toes_right";
            tr3.transform.translation.x = TRANS_X;
            tr3.transform.translation.y = 0.0;
            tr3.transform.translation.z = TRANS_Z;
            tr3.transform.rotation = Quaternion();
            tf_broadcaster_->sendTransform(tr3);

            // Transformation from right toes to right aligned
            TransformStamped tr4;
            tr4.header.stamp = trans_right_.header.stamp;
            tr4.header.frame_id = "toes_right";
            tr4.child_frame_id = "toes_right_aligned";
            tr4.transform.rotation = trans_right_.transform.rotation;
            tf_broadcaster_->sendTransform(tr4);

            last_published_right_ = trans_right_.header.stamp;
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<FramePublisherNode>();
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
