#include "state_estimator.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

StateEstimator::StateEstimator()
    : Node("state_estimator_node")
    , m_joint_estimator(this, get_initial_joint_states())
    , m_com_estimator(this)
{
    m_state_publisher = this->create_publisher<march_shared_msgs::msg::RobotState>("robot_state", 10);
    m_sensor_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 10, std::bind(&StateEstimator::sensor_callback, this, _1));
    m_state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_state", 10, std::bind(&StateEstimator::state_callback, this, _1));
    m_tf_joint_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    m_com_pos_publisher = this->create_publisher<geometry_msgs::msg::PointStamped>("robot_com_position", 100);

    timer_ = this->create_wall_timer(1000ms, std::bind(&StateEstimator::publish_robot_frames, this));

    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_joint_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
}

sensor_msgs::msg::JointState StateEstimator::get_initial_joint_states()
{
    sensor_msgs::msg::JointState initial_joint_state;
    // change it so the names are obtained from the parameter
    initial_joint_state.name = { "right_ankle", "right_knee", "right_hip_fe", "right_hip_aa", "left_ankle", "left_knee",
        "left_hip_fe", "left_hip_aa", "right_origin", "left_origin" };
    initial_joint_state.position = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    return initial_joint_state;
}

void StateEstimator::sensor_callback(sensor_msgs::msg::Imu::SharedPtr msg)
{
}

void StateEstimator::state_callback(sensor_msgs::msg::JointState::SharedPtr msg)
{
}

void StateEstimator::publish_robot_state()
{
    auto msg = march_shared_msgs::msg::RobotState();
    msg.stamp = this->get_clock()->now();
    msg.joint_names.push_back("");
    msg.joint_pos.push_back(0);
    msg.joint_vel.push_back(0);
    msg.sensor_names.push_back("");
    msg.sensor_data.push_back(0);

    m_state_publisher->publish(msg);
}

void StateEstimator::publish_robot_frames()
{
    m_joint_estimator.set_individual_joint_state("right_ankle", 3.14);
    RCLCPP_INFO(this->get_logger(), "Number of frames is %i", m_joint_estimator.get_joint_frames().size());
    for (auto i : m_joint_estimator.get_joint_frames()) {
        m_tf_joint_broadcaster->sendTransform(i);
        RCLCPP_INFO(this->get_logger(),
            ("\n Set up link " + i.header.frame_id + "\n with child link " + i.child_frame_id).c_str());
    }
    std::vector<CenterOfMass> test = m_joint_estimator.get_joint_com_positions("right_knee");
    RCLCPP_INFO(this->get_logger(), "Array size is %i", test.size());
    for (auto com : test) {
        RCLCPP_INFO(this->get_logger(), ("\n Publishing COM"));
        RCLCPP_INFO(this->get_logger(), "\n Publishing COM with pos x = %f", com.position.point.x);
        m_com_pos_publisher->publish(com.position);
    }
    // RCLCPP_INFO(this->get_logger(), "Test amount: %i", test[0].point.x);
}

geometry_msgs::msg::TransformStamped StateEstimator::get_frame_transform(
    std::string& target_frame, std::string& source_frame)
{
    geometry_msgs::msg::TransformStamped frame_transform;
    try {
        frame_transform = m_tf_buffer->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
        return frame_transform;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "error in get_frame_transform: %s", ex.what());
        return frame_transform;
    }
}

geometry_msgs::msg::Point transform_point(
    std::string& target_frame, std::string& source_frame, geometry_msgs::msg::Point& point_to_transform)
{
}