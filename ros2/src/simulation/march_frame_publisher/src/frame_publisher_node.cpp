#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <std_srvs/srv/trigger.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include "march_shared_msgs/msg/foot_position.hpp"
#include <thread>
#include <chrono>

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

        service_ = this->create_service<std_srvs::srv::Trigger>("~/align_cameras", std::bind(&FramePublisherNode::alignCameras, this, std::placeholders::_1, std::placeholders::_2));
        vision_parameter_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "march_foot_position_finder");

        left_point_subscriber_
        = this->create_subscription<march_shared_msgs::msg::FootPosition>(
            "/march/chosen_foot_position/left",
            /*qos=*/1,
            std::bind(&FramePublisherNode::leftPointCallback, this,
                std::placeholders::_1));

        right_point_subscriber_
        = this->create_subscription<march_shared_msgs::msg::FootPosition>(
            "/march/chosen_foot_position/right",
            /*qos=*/1,
            std::bind(&FramePublisherNode::rightPointCallback, this,
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

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
    std::shared_ptr<rclcpp::SyncParametersClient> vision_parameter_client_;

    rclcpp::Subscription<march_shared_msgs::msg::FootPosition>::SharedPtr left_point_subscriber_;
    rclcpp::Subscription<march_shared_msgs::msg::FootPosition>::SharedPtr right_point_subscriber_;

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

    geometry_msgs::msg::Point last_left_point_;
    geometry_msgs::msg::Point last_right_point_;
    int last_left_point_count_ = 0;
    int last_right_point_count_ = 0;

    int average_count_ = 4;

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
        for (const rclcpp::Parameter& param : parameters) {
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

    void leftPointCallback(
    const march_shared_msgs::msg::FootPosition::SharedPtr msg)
    {
        last_left_point_ = msg->displacement;
        last_left_point_count_++;
    }

    void rightPointCallback(
        const march_shared_msgs::msg::FootPosition::SharedPtr msg)
    {
        last_right_point_ = msg->displacement;
        last_right_point_count_++;
    }

    void parameterUpdatedLogger(const rclcpp::Parameter& param)
    {
        RCLCPP_INFO(this->get_logger(),
            param.get_name() + " set to " + param.value_to_string());
    }

    void publishCameraFrame(const std::string& left_or_right)
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
            } else {
                tf2_quaternion.setRPY(0.0, 0.0, 0.0);
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

    void alignCameras(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void) request; // Silence unused warning. The request parameter does not contain any data.

        auto original_parameter = vision_parameter_client_->get_parameters({"zero_height_threshold"});
        std::vector<rclcpp::Parameter> set_param = {rclcpp::Parameter("zero_height_threshold", 0.0)};
        vision_parameter_client_->set_parameters(set_param);

        double optimal_left_angle = 0.0, optimal_right_angle = 0.0;
        double closest_left_height_to_zero = 1000, closest_right_height_to_zero = 1000;

        for (double angle = -7.0; angle <= 7.0; angle += 0.1) {

            std::cout << "Checking angle: " << angle << std::endl;

            double avg_left_height = 0.0, avg_right_height = 0.0;
            int start_left_index, start_right_index;

            rotation_camera_left = angle;
            start_left_index = last_left_point_count_;

            rotation_camera_right = angle;
            start_right_index = last_right_point_count_;

            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            for (int i = 0; i < average_count_; i++) {
                while (last_left_point_count_ == start_left_index) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
                avg_left_height += last_left_point_.z;
                start_left_index = last_left_point_count_;
            }
            
            for (int i = 0; i < average_count_; i++) {
                while (last_right_point_count_ == start_right_index) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
                avg_right_height += last_right_point_.z;
                start_right_index = last_right_point_count_;
            }

            avg_right_height /= average_count_;
            avg_left_height /= average_count_;

            if (abs(avg_left_height) < abs(closest_left_height_to_zero)) {
                closest_left_height_to_zero = avg_left_height;
                optimal_left_angle = angle;
            }


            if (abs(avg_right_height) < abs(closest_right_height_to_zero)) {
                closest_right_height_to_zero = avg_right_height;
                optimal_right_angle = angle;
            }
        }

        this->set_parameter(rclcpp::Parameter("rotation_camera_left", optimal_left_angle));
        this->set_parameter(rclcpp::Parameter("rotation_camera_right", optimal_right_angle));

        std::vector<rclcpp::Parameter> reset_param = {rclcpp::Parameter("zero_height_threshold", original_parameter[0].as_double())};
        vision_parameter_client_->set_parameters(reset_param);

        response->success = true;
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
