/**
 * @author Jelmer de Wolde, Tuhin Das - MARCH 7
 */

#include <frame_publisher.h>

FramePublisher::FramePublisher()
    : Node("march_frame_publisher",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
            /*automatically_declare_parameters_from_overrides=*/true))
{
    client_node_ = std::make_shared<rclcpp::Node>("frame_publisher_client_node");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    left_aligner_ = std::make_unique<PointCloudAligner>(this, "left");
    right_aligner_ = std::make_unique<PointCloudAligner>(this, "right");

    timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(5),
        [this]() -> void {
            publishToeFrames();
            publishCameraFrame("left");
            publishCameraFrame("right");
        },
        timer_group_);

    callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&FramePublisher::parametersCallback, this, std::placeholders::_1));

    align_service_ = this->create_service<std_srvs::srv::Trigger>("~/align_cameras",
        std::bind(&FramePublisher::alignCameras, this, std::placeholders::_1, std::placeholders::_2));

    get_threshold_client_ = client_node_->create_client<rcl_interfaces::srv::GetParameters>(
        "/march/march_foot_position_finder/get_parameters");
    set_threshold_client_ = client_node_->create_client<rcl_interfaces::srv::SetParameters>(
        "/march/march_foot_position_finder/set_parameters");

    rotation_camera_left_ = this->get_parameter("rotation_camera_left").as_double();
    rotation_camera_right_ = this->get_parameter("rotation_camera_right").as_double();
}

/**
 * Update parameter with dynamic reconfigure.
 */
rcl_interfaces::msg::SetParametersResult FramePublisher::parametersCallback(
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
        parameterUpdatedLogger(param);
    }
    left_aligner_->parametersCallback(parameters);
    right_aligner_->parametersCallback(parameters);
    return result;
}

/**
 * Notify which parameter was updated.
 *
 * @param param Parameter that was updated.
 */
void FramePublisher::parameterUpdatedLogger(const rclcpp::Parameter& param)
{
    RCLCPP_INFO(this->get_logger(), param.get_name() + " set to " + param.value_to_string());
}

/**
 * Publish a frame similar to the depth camera frame, only rotated around an
 * angle. This angle is used to align depth frames horizontally.
 *
 * @param left_or_right Left or right depth frame to transform and publish.
 */
void FramePublisher::publishCameraFrame(const std::string& left_or_right)
{
    tr.header.stamp = this->get_clock()->now();
    if ((left_or_right == "left" && rclcpp::Time(tr.header.stamp) > last_published_camera_left_)
        || (left_or_right == "right" && rclcpp::Time(tr.header.stamp) > last_published_camera_right_)) {
        tr.header.frame_id = "camera_front_" + left_or_right + "_depth_optical_frame";
        tr.child_frame_id = "camera_front_" + left_or_right + "_virtual_rotated";
        tf2::Quaternion tf2_quaternion;
        if (left_or_right == "left") {
            tf2_quaternion.setRPY(rotation_camera_left_ / 180 * M_PI, 0.0, 0.0);
            last_published_camera_left_ = rclcpp::Time(tr.header.stamp);
        } else if (left_or_right == "right") {
            tf2_quaternion.setRPY(rotation_camera_right_ / 180 * M_PI, 0.0, 0.0);
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
void FramePublisher::publishToeFrames()
{

    using namespace geometry_msgs::msg;

    try {
        trans_left_ = tf_buffer_->lookupTransform("foot_left", "world", tf2::TimePointZero);
        trans_right_ = tf_buffer_->lookupTransform("foot_right", "world", tf2::TimePointZero);

        if (!start_time_initialized_) {
            last_published_left_ = trans_left_.header.stamp;
            last_published_right_ = trans_right_.header.stamp;
            start_time_initialized_ = true;
        }

    } catch (tf2::TransformException& ex) {
        return;
    }

    if (rclcpp::Time(trans_left_.header.stamp) > rclcpp::Time(last_published_left_)) {
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

    if (rclcpp::Time(trans_right_.header.stamp) > rclcpp::Time(last_published_right_)) {
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
void FramePublisher::alignCamerasCallback()
{
    double original_threshold = getHeightZeroThreshold();
    setHeightZeroThreshold(/*threshold=*/0.0);

    std::thread thread_left([this] {
        left_aligner_->alignPointCloud();
    });
    std::thread thread_right([this] {
        right_aligner_->alignPointCloud();
    });

    thread_left.join();
    thread_right.join();

    setHeightZeroThreshold(original_threshold);
}

/**
 * Callback for when the camera align service is called. The actual
 * alignment is run in an asynchronous thread.
 *
 * @param request Service request message.
 * @param response Service response message.
 */
void FramePublisher::alignCameras(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request, // NOLINT, binding does not expect reference
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response) // NOLINT, binding does not expect reference
{
    (void)request; // Silence unused warning. Request has no data.
    std::thread thread(&FramePublisher::alignCamerasCallback, this);
    thread.detach();
    response->success = true;
}

/**
 * Retrieve the current height zero threshold from the foot position finder
 * node, using the get_parameter service.
 *
 * @return double The current height zero threshold.
 */
double FramePublisher::getHeightZeroThreshold()
{
    double threshold = 0.0;
    auto get_request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    get_request->names.push_back("height_zero_threshold");
    while (!get_threshold_client_->wait_for_service(1s)) { }

    auto get_result = get_threshold_client_->async_send_request(get_request);

    if (rclcpp::spin_until_future_complete(client_node_->get_node_base_interface(), get_result)
        == rclcpp::FutureReturnCode::SUCCESS) {
        threshold = get_result.get()->values[0].double_value;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to get original zero height threshold.");
    }
    return threshold;
}

/**
 * Set the current height zero threshold in the foot position finder node,
 * using the set_parameter service.
 *
 * @param threshold The new threshold.
 */
void FramePublisher::setHeightZeroThreshold(double threshold)
{
    auto set_request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

    rcl_interfaces::msg::ParameterValue value;
    value.type = 3;
    value.double_value = threshold;

    rcl_interfaces::msg::Parameter param;
    param.name = "height_zero_threshold";
    param.value = value;
    set_request->parameters.push_back(param);

    while (!set_threshold_client_->wait_for_service(1s)) { }
    auto set_result = set_threshold_client_->async_send_request(set_request);
    if (rclcpp::spin_until_future_complete(client_node_->get_node_base_interface(), set_result)
        != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to automatically set zero height threshold.");
    };
}

/**
 * Set the left camera rotation.
 *
 * @param angle Angle to set.
 */
void FramePublisher::setLeftCameraRotation(double angle)
{
    rotation_camera_left_ = angle;
}

/**
 * Set the right camera rotation.
 *
 * @param angle Angle to set.
 */
void FramePublisher::setRightCameraRotation(double angle)
{
    rotation_camera_right_ = angle;
}
