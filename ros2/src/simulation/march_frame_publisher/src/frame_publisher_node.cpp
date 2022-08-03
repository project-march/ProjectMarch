#include "march_shared_msgs/msg/foot_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <thread>

using namespace std::chrono_literals;
using TransformStamped = geometry_msgs::msg::TransformStamped;

class FramePublisherNode : public rclcpp::Node {
public:
    FramePublisherNode()
        : Node("march_frame_publisher")
    {
        client_node_
            = std::make_shared<rclcpp::Node>("frame_publisher_client_node");
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_
            = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_
            = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        timer_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),
            [this]() -> void {
                publishToeFrames();
                publishCameraFrame("left");
                publishCameraFrame("right");
            },
            timer_group_);

        rclcpp::SubscriptionOptions options;
        options.callback_group = callback_group_;

        callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&FramePublisherNode::parametersCallback, this,
                std::placeholders::_1));

        align_service_
            = this->create_service<std_srvs::srv::Trigger>("~/align_cameras",
                std::bind(&FramePublisherNode::alignCameras, this,
                    std::placeholders::_1, std::placeholders::_2));

        get_threshold_client_
            = client_node_->create_client<rcl_interfaces::srv::GetParameters>(
                "/march/march_foot_position_finder/get_parameters");
        set_threshold_client_
            = client_node_->create_client<rcl_interfaces::srv::SetParameters>(
                "/march/march_foot_position_finder/set_parameters");

        left_point_subscriber_
            = this->create_subscription<march_shared_msgs::msg::FootPosition>(
                "/march/foot_position/left",
                /*qos=*/1,
                std::bind(&FramePublisherNode::leftPointCallback, this,
                    std::placeholders::_1),
                options);

        right_point_subscriber_
            = this->create_subscription<march_shared_msgs::msg::FootPosition>(
                "/march/foot_position/right",
                /*qos=*/1,
                std::bind(&FramePublisherNode::rightPointCallback, this,
                    std::placeholders::_1),
                options);

        range_.set__from_value(-10.0).set__to_value(10.0).set__step(0.1);
        descriptor_.floating_point_range = { range_ };
        this->declare_parameter<double>(
            "rotation_camera_left", rotation_camera_left_, descriptor_);
        this->declare_parameter<double>(
            "rotation_camera_right", rotation_camera_right_, descriptor_);
        this->declare_parameter<double>("angle_offset", angle_offset_);
        this->declare_parameter<double>("min_check_angle", min_check_angle_);
        this->declare_parameter<double>("max_check_angle", max_check_angle_);
    }

private:
    std::shared_ptr<rclcpp::Node> client_node_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_ { nullptr };
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr align_service_;
    std::shared_ptr<rclcpp::SyncParametersClient> vision_parameter_client_;

    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr
        get_threshold_client_;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr
        set_threshold_client_;

    rclcpp::Subscription<march_shared_msgs::msg::FootPosition>::SharedPtr
        left_point_subscriber_;
    rclcpp::Subscription<march_shared_msgs::msg::FootPosition>::SharedPtr
        right_point_subscriber_;

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
    bool start_time_initialized_ = false;

    double rotation_camera_left_ = 0.0;
    double rotation_camera_right_ = 0.0;

    rclcpp::CallbackGroup::SharedPtr timer_group_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;

    geometry_msgs::msg::Point last_left_point_;
    geometry_msgs::msg::Point last_right_point_;
    int last_left_point_count_ = 0;
    int last_right_point_count_ = 0;

    int average_count_ = 4;
    int skip_point_num_ = 2;
    double min_check_angle_ = -5.0;
    double max_check_angle_ = 5.0;
    double angle_offset_ = 0.2;

    TransformStamped tr;
    rclcpp::Time last_published_camera_left_ = tr.header.stamp;
    rclcpp::Time last_published_camera_right_ = tr.header.stamp;

    rcl_interfaces::msg::ParameterDescriptor descriptor_;
    rcl_interfaces::msg::FloatingPointRange range_;

    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    /**
     * Update parameter with dynamic reconfigure.
     */
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter>& parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        for (const rclcpp::Parameter& param : parameters) {
            if (param.get_name() == "rotation_camera_left") {
                rotation_camera_left_ = param.as_double();
            }
            if (param.get_name() == "rotation_camera_right") {
                rotation_camera_right_ = param.as_double();
            }
            if (param.get_name() == "angle_offset") {
                angle_offset_ = param.as_double();
            }
            if (param.get_name() == "min_check_angle") {
                min_check_angle_ = param.as_double();
            }
            if (param.get_name() == "max_check_angle") {
                max_check_angle_ = param.as_double();
            }
            parameterUpdatedLogger(param);
        }
        return result;
    }

    /**
     * @brief Store the latest found left camera foot position.
     *
     * @param msg Message containing the foot position.
     */
    void leftPointCallback(const march_shared_msgs::msg::FootPosition::SharedPtr
            msg) // NOLINT, binding does not expect reference
    {
        last_left_point_ = msg->displacement;
        last_left_point_count_++;
    }

    /**
     * @brief Store the latest found right camera foot position.
     *
     * @param msg Message containing the foot position.
     */
    void rightPointCallback(
        const march_shared_msgs::msg::FootPosition::SharedPtr
            msg) // NOLINT, binding does not expect reference
    {
        last_right_point_ = msg->displacement;
        last_right_point_count_++;
    }

    /**
     * Notify which parameter was updated.
     *
     * @param param Parameter that was updated.
     */
    void parameterUpdatedLogger(const rclcpp::Parameter& param)
    {
        RCLCPP_INFO(this->get_logger(),
            param.get_name() + " set to " + param.value_to_string());
    }

    /**
     * Publish a frame similar to the depth camera frame, only rotated around an
     * angle. This angle is used to align depth frames horizontally.
     *
     * @param left_or_right Left or right depth frame to transform and publish.
     */
    void publishCameraFrame(const std::string& left_or_right)
    {
        tr.header.stamp = this->get_clock()->now();
        if ((left_or_right == "left"
                && rclcpp::Time(tr.header.stamp) > last_published_camera_left_)
            || (left_or_right == "right"
                && rclcpp::Time(tr.header.stamp)
                    > last_published_camera_right_)) {
            tr.header.frame_id
                = "camera_front_" + left_or_right + "_depth_optical_frame";
            tr.child_frame_id
                = "camera_front_" + left_or_right + "_virtual_rotated";
            tf2::Quaternion tf2_quaternion;
            if (left_or_right == "left") {
                tf2_quaternion.setRPY(
                    rotation_camera_left_ / 180 * M_PI, 0.0, 0.0);
                last_published_camera_left_ = rclcpp::Time(tr.header.stamp);
            } else if (left_or_right == "right") {
                tf2_quaternion.setRPY(
                    rotation_camera_right_ / 180 * M_PI, 0.0, 0.0);
                last_published_camera_right_ = rclcpp::Time(tr.header.stamp);
            } else {
                tf2_quaternion.setRPY(0.0, 0.0, 0.0);
            }

            tr.transform.rotation = tf2::toMsg(tf2_quaternion);
            tf_broadcaster_->sendTransform(tr);
        }
    }

    /**
     * Publish toe framed aligned with the world frame, so camera points can be
     * considered relative to the current toe positions.
     */
    void publishToeFrames()
    {

        using namespace geometry_msgs::msg;

        try {
            trans_left_ = tf_buffer_->lookupTransform(
                "foot_left", "world", tf2::TimePointZero);
            trans_right_ = tf_buffer_->lookupTransform(
                "foot_right", "world", tf2::TimePointZero);

            if (!start_time_initialized_) {
                last_published_left_ = trans_left_.header.stamp;
                last_published_right_ = trans_right_.header.stamp;
                start_time_initialized_ = true;
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

    /**
     * Align the pointclouds of the two cameras by rotating them around a
     * variable angle. For both the left and right camera an optimal rotation
     * angle is chosen so that points on the ground have a observed height as
     * close as possible to 0.
     */
    void alignCamerasCallback()
    {
        double original_threshold = getHeightZeroThreshold();
        setHeightZeroThreshold(/*threshold=*/0.0);

        double optimal_left_angle = 0.0, optimal_right_angle = 0.0;
        double closest_left_height_to_zero = 1000,
               closest_right_height_to_zero = 1000;

        // double is useful in this loop, so turn off 'should be int' lint
        // NOLINTNEXTLINE
        for (double angle = min_check_angle_; angle <= max_check_angle_;
             // NOLINTNEXTLINE
             angle += angle_offset_) {

            RCLCPP_INFO(this->get_logger(), "Checking angle %lf", angle);

            double avg_left_height = 0.0, avg_right_height = 0.0;
            int start_left_index, start_right_index;

            rotation_camera_left_ = angle;
            start_left_index = last_left_point_count_;

            rotation_camera_right_ = angle;
            start_right_index = last_right_point_count_;

            int skip_points = 0;
            while (skip_points < skip_point_num_) {
                if (last_left_point_count_ != start_left_index) {
                    last_left_point_count_ = start_left_index;
                    skip_points++;
                }
            }

            for (int i = 0; i < average_count_; i++) {
                while (last_left_point_count_ == start_left_index) {
                }
                avg_left_height += last_left_point_.z;
                start_left_index = last_left_point_count_;
            }

            for (int i = 0; i < average_count_; i++) {
                while (last_right_point_count_ == start_right_index) {
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

        this->set_parameter(
            rclcpp::Parameter("rotation_camera_left", optimal_left_angle));
        this->set_parameter(
            rclcpp::Parameter("rotation_camera_right", optimal_right_angle));

        setHeightZeroThreshold(original_threshold);
    }

    /**
     * Callback for when the camera align service is called. The actual
     * alignment is run in an asynchronous thread.
     *
     * @param request Service request message.
     * @param response Service response message.
     */
    void alignCameras(const std::shared_ptr<std_srvs::srv::Trigger::Request>
                          request, // NOLINT, binding does not expect reference
        const std::shared_ptr<std_srvs::srv::Trigger::Response>
            response) // NOLINT, binding does not expect reference
    {
        (void)request; // Silence unused warning. Request has no data.
        std::thread thread(&FramePublisherNode::alignCamerasCallback, this);
        thread.detach();
        response->success = true;
    }

    /**
     * Retrieve the current height zero threshold from the foot position finder
     * node, using the get_parameter service.
     *
     * @return double The current height zero threshold.
     */
    double getHeightZeroThreshold()
    {
        double threshold = 0.0;
        auto get_request
            = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
        get_request->names.push_back("height_zero_threshold");
        while (!get_threshold_client_->wait_for_service(1s)) {
        }

        auto get_result
            = get_threshold_client_->async_send_request(get_request);

        if (rclcpp::spin_until_future_complete(
                client_node_->get_node_base_interface(), get_result)
            == rclcpp::FutureReturnCode::SUCCESS) {
            threshold = get_result.get()->values[0].double_value;
        } else {
            RCLCPP_ERROR(this->get_logger(),
                "Failed to get original zero height threshold.");
        }
        return threshold;
    }

    /**
     * Set the current height zero threshold in the foot position finder node,
     * using the set_parameter service.
     *
     * @param threshold The new threshold.
     */
    void setHeightZeroThreshold(double threshold)
    {
        auto set_request
            = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

        rcl_interfaces::msg::ParameterValue value;
        value.type = 3;
        value.double_value = threshold;

        rcl_interfaces::msg::Parameter param;
        param.name = "height_zero_threshold";
        param.value = value;
        set_request->parameters.push_back(param);

        while (!set_threshold_client_->wait_for_service(1s)) {
        }
        auto set_result
            = set_threshold_client_->async_send_request(set_request);
        if (rclcpp::spin_until_future_complete(
                client_node_->get_node_base_interface(), set_result)
            != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(),
                "Failed to automatically set zero height threshold.");
        };
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
