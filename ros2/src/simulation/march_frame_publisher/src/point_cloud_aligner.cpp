/**
 * @author Tuhin Das - MARCH 7
 */

#include <point_cloud_aligner.h>

PointCloudAligner::PointCloudAligner(
    FramePublisher* n, std::string left_or_right)
    : n_ { n }
    , left_or_right_ { std::move(left_or_right) }
{
    point_count_ = 0;

    min_check_angle_ = n_->get_parameter("min_check_angle").as_double();
    max_check_angle_ = n_->get_parameter("max_check_angle").as_double();
    angle_offset_ = n_->get_parameter("angle_offset").as_double();
    avg_sample_size_ = n_->get_parameter("avg_sample_size").as_int();
    num_skip_points_ = n_->get_parameter("num_skip_points").as_int();
    binary_steps_ = n_->get_parameter("binary_steps").as_int();
    binary_search_ = n_->get_parameter("binary_search").as_bool();

    callback_group_
        = n_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;

    point_subscriber_
        = n_->create_subscription<march_shared_msgs::msg::FootPosition>(
            "/march/foot_position/" + left_or_right_,
            /*qos=*/1,
            std::bind(
                &PointCloudAligner::pointCallback, this, std::placeholders::_1),
            options);
}
/**
 * @brief Store the latest found camera foot position, and keep track of the
 * total number of found points.
 *
 * @param msg Message containing the foot position.
 */
void PointCloudAligner::pointCallback(
    const march_shared_msgs::msg::FootPosition::SharedPtr
        msg) // NOLINT, binding does not expect reference
{
    point_count_++;
    last_point_ = msg->displacement;
}

/**
 * Align the left or right pointcloud by rotating them around a certain angle.
 * The optimal angle is found with a linear or binary search.
 */
void PointCloudAligner::alignPointCloud()
{
    double optimal_angle = 0.0;
    if (binary_search_) {
        optimal_angle = binarySearchOptimalAngle();
    } else {
        optimal_angle = linearSearchOptimalAngle();
    }

    if (left_or_right_ == "left") {
        n_->set_parameter(
            rclcpp::Parameter("rotation_camera_left", optimal_angle));
    } else {
        n_->set_parameter(
            rclcpp::Parameter("rotation_camera_right", optimal_angle));
    }
}

/**
 * Try a range of rotation angles with a binary search algorithm to find the
 * angle for which the found points have a height (z-value) closest to zero.
 *
 * @return double Optimal angle to find points with zero height.
 */
double PointCloudAligner::binarySearchOptimalAngle()
{
    double bounds[3] = { min_check_angle_,
        0.5 * (min_check_angle_ + max_check_angle_), max_check_angle_ };
    double heights[3] = { computeAverageHeight(bounds[0]),
        computeAverageHeight(bounds[1]), computeAverageHeight(bounds[2]) };

    double optimal_angle = bounds[0];
    double optimal_height = std::abs(heights[1]);

    for (int i = 0; i < binary_steps_; i++) {
        RCLCPP_INFO(n_->get_logger(), "Step %d of %d of binary search (%s)",
            i + 1, binary_steps_, left_or_right_.c_str());

        if ((heights[1] < 0 && 0 < heights[2])
            || (heights[1] > 0 && 0 > heights[2])) {
            bounds[0] = bounds[1];
            heights[0] = heights[1];
        } else if ((heights[1] > 0 && 0 > heights[0])
            || (heights[1] < 0 && 0 < heights[0])) {
            bounds[2] = bounds[1];
            heights[2] = heights[1];
        } else {
            break;
        }

        bounds[1] = 0.5 * (bounds[0] + bounds[2]);
        heights[1] = computeAverageHeight(bounds[1]);

        if (abs(heights[1]) < abs(optimal_height)) {
            optimal_height = heights[1];
            optimal_angle = bounds[1];
        }
    }

    RCLCPP_INFO(n_->get_logger(),
        "Zero height found using binary search: %lf (%s)", optimal_height,
        left_or_right_.c_str());

    return optimal_angle;
}

/**
 * Try a range of rotation angles in a linear fashion, and compute for which
 * angle the found points have a height (z-value) closest to zero.
 *
 * @return double Optimal angle to find points with zero height.
 */
double PointCloudAligner::linearSearchOptimalAngle()
{

    double optimal_angle = 0.0;
    double optimal_height = 1000;

    // double is useful in this loop, so turn off 'should be int' lint
    // NOLINTNEXTLINE
    for (double angle = min_check_angle_; angle <= max_check_angle_;
         // NOLINTNEXTLINE
         angle += angle_offset_) {

        RCLCPP_INFO(n_->get_logger(), "Checking angle %lf (%s)", angle,
            left_or_right_.c_str());

        double height = computeAverageHeight(angle);

        if (abs(height) < abs(optimal_height)) {
            optimal_height = height;
            optimal_angle = angle;
        }
    }

    RCLCPP_INFO(n_->get_logger(),
        "Zero height found using linear search: %lf (%s)", optimal_height,
        left_or_right_.c_str());

    return optimal_angle;
}

/**
 * Update parameter with dynamic reconfigure.
 */
double PointCloudAligner::computeAverageHeight(double angle)
{
    double avg_height = 0.0;
    int start_index = point_count_;
    if (left_or_right_ == "left") {
        n_->setLeftCameraRotation(angle);
    } else {
        n_->setRightCameraRotation(angle);
    }

    int skip_points = 0;

    while (skip_points < num_skip_points_) {
        if (point_count_ != start_index) {
            point_count_ = start_index;
            skip_points++;
        }
    }

    for (int i = 0; i < avg_sample_size_; i++) {
        while (point_count_ == start_index) {
        }
        avg_height += last_point_.z;
        start_index = point_count_;
    }

    return avg_height / avg_sample_size_;
}

/**
 * Update parameters with dynamic reconfigure.
 */
rcl_interfaces::msg::SetParametersResult PointCloudAligner::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const rclcpp::Parameter& param : parameters) {
        if (param.get_name() == "angle_offset") {
            angle_offset_ = param.as_double();
        }
        if (param.get_name() == "min_check_angle") {
            min_check_angle_ = param.as_double();
        }
        if (param.get_name() == "max_check_angle") {
            max_check_angle_ = param.as_double();
        }
        if (param.get_name() == "avg_sample_size") {
            avg_sample_size_ = param.as_int();
        }
        if (param.get_name() == "num_skip_points") {
            num_skip_points_ = param.as_int();
        }
        if (param.get_name() == "binary_steps") {
            binary_steps_ = param.as_int();
        }
        if (param.get_name() == "binary_search") {
            binary_search_ = param.as_bool();
        }
    }
    return result;
}

/**
 * Notify which parameter was updated.
 *
 * @param param Parameter that was updated.
 */
void PointCloudAligner::parameterUpdatedLogger(const rclcpp::Parameter& param)
{
    RCLCPP_INFO(n_->get_logger(),
        param.get_name() + " set to " + param.value_to_string());
}