/**
 * @author Tuhin Das - MARCH 7
 */

#include <point_cloud_aligner.h>

PointCloudAligner::PointCloudAligner(
    FramePublisher* n, std::string left_or_right)
    : n_ { n }
    , left_or_right_ { std::move(left_or_right) }
{
    last_point_count_ = 0;

    min_check_angle_ = n_->get_parameter("min_check_angle").as_double();
    max_check_angle_ = n_->get_parameter("max_check_angle").as_double();
    angle_offset_ = n_->get_parameter("angle_offset").as_double();
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
 * @brief Store the latest found left camera foot position.
 *
 * @param msg Message containing the foot position.
 */
void PointCloudAligner::pointCallback(
    const march_shared_msgs::msg::FootPosition::SharedPtr
        msg) // NOLINT, binding does not expect reference
{
    last_point_count_++;
    last_point_ = msg->displacement;
}

void PointCloudAligner::alignPointCloud()
{
    double optimal_angle = 0.0;
    if (binary_search_) {
        optimal_angle = binarySearchOptimalAngle();
    } else {
        optimal_angle = linearSearchOptimalAngle();
    }

    if (left_or_right_ == "left") {
        n_->set_parameter(rclcpp::Parameter("rotation_camera_left", optimal_angle));
    } else {
        n_->set_parameter(
            rclcpp::Parameter("rotation_camera_right", optimal_angle));
    }
}

double PointCloudAligner::binarySearchOptimalAngle()
{
    double bounds[3] = { min_check_angle_,
        0.5 * (min_check_angle_ + max_check_angle_), max_check_angle_ };
    double heights[3] = { computeAverageHeight(bounds[0]),
        computeAverageHeight(bounds[1]), computeAverageHeight(bounds[2]) };

    for (int i = 0; i < 10; i++) {
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
    } 

    if (left_or_right_ == "left") {
        RCLCPP_INFO(n_->get_logger(), "Zero height found using binary search: %lf (left)", heights[1]);
    } else {
        RCLCPP_INFO(n_->get_logger(), "Zero height found using binary search: %lf (right)", heights[1]);
    }

    return bounds[1];
}

double PointCloudAligner::linearSearchOptimalAngle()
{

    double optimal_angle = 0.0;
    double optimal_height = 1000;

    // double is useful in this loop, so turn off 'should be int' lint
    // NOLINTNEXTLINE
    for (double angle = min_check_angle_; angle <= max_check_angle_;
            // NOLINTNEXTLINE
            angle += angle_offset_) {

        double height = computeAverageHeight(angle);
        
        if (abs(height) < abs(optimal_height)) {
            optimal_height = height;
            optimal_angle = angle;
        }
    }

    if (left_or_right_ == "left") {
        RCLCPP_INFO(n_->get_logger(), "Zero height found using linear search: %lf (left)", optimal_height);
    } else {
        RCLCPP_INFO(n_->get_logger(), "Zero height found using linear search: %lf (right)", optimal_height);
    }
                
    return optimal_angle;
}

double PointCloudAligner::computeAverageHeight(double angle)
{
    RCLCPP_INFO(n_->get_logger(), "Checking angle %lf", angle);
    return 0.0;

    double avg_height = 0.0;
    int start_index = last_point_count_;
    if (left_or_right_ == "left") {
        n_->setLeftCameraRotation(angle);
    } else {
        n_->setRightCameraRotation(angle);
    }

    int skip_points = 0;

    while (skip_points < skip_point_num_) {
        if (last_point_count_ != start_index) {
            last_point_count_ = start_index;
            skip_points++;
        }
    }

    for (int i = 0; i < average_count_; i++) {
        while (last_point_count_ == start_index) {
        }
        avg_height += last_point_.z;
        start_index = last_point_count_;
    }

    avg_height /= average_count_;

    return avg_height;
}

/**
 * Update parameter with dynamic reconfigure.
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
        if (param.get_name() == "binary_search") {
            binary_search_ = param.as_bool();
        }
    }
    return result;
}