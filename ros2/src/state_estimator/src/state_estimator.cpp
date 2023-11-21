#include "state_estimator/state_estimator.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

StateEstimator::StateEstimator()
    : Node("state_estimator_node")
    , m_joint_estimator(this)
    , m_imu_estimator()
    , m_current_stance_foot(-1)
{
    m_upper_imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
        "/upper_imu", 10, std::bind(&StateEstimator::upper_imu_callback, this, _1));
    m_lower_imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
        "/lower_imu", 10, std::bind(&StateEstimator::lower_imu_callback, this, _1));
    m_state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&StateEstimator::state_callback, this, _1));

    m_tf_joint_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    m_com_pos_publisher = this->create_publisher<march_shared_msgs::msg::CenterOfMass>("robot_com_position", 100);

    m_stance_foot_publisher = this->create_publisher<std_msgs::msg::Int32>("current_stance_foot", 100);

    m_right_foot_on_ground_publisher = this->create_publisher<std_msgs::msg::Bool>("right_foot_on_ground", 100);

    m_left_foot_on_ground_publisher = this->create_publisher<std_msgs::msg::Bool>("left_foot_on_ground", 100);

    m_foot_impact_publisher = this->create_publisher<march_shared_msgs::msg::Feet>("foot_impact", 100);

    m_feet_height_publisher
        = this->create_publisher<march_shared_msgs::msg::FeetHeightStamped>("robot_feet_height", 100);

    m_joint_state_publisher = this->create_publisher<sensor_msgs::msg::JointState>("measured_joint_states", 100);

    m_rviz_publisher = this->create_publisher<visualization_msgs::msg::Marker>("joint_visualizations", 100);

    declare_parameter("state_estimator_config.refresh_rate", 1000);
    auto refresh_rate = this->get_parameter("state_estimator_config.refresh_rate").as_int();
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(refresh_rate), std::bind(&StateEstimator::publish_robot_frames, this));

    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_joint_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

    // Declare parameters
    // IMU parameters
    declare_parameter("imu_estimator.lower_imu.IMU_exo_base_link", std::string("default"));
    declare_parameter("imu_estimator.lower_imu.IMU_exo_position", std::vector<double>(3, 0.0));
    declare_parameter("imu_estimator.lower_imu.IMU_exo_rotation", std::vector<double>(3, 0.0));
    declare_parameter("imu_estimator.upper_imu.IMU_exo_base_link", std::string("default"));
    declare_parameter("imu_estimator.upper_imu.IMU_exo_position", std::vector<double>(3, 0.0));
    declare_parameter("imu_estimator.upper_imu.IMU_exo_rotation", std::vector<double>(3, 0.0));

    declare_parameter("footstep_estimator.left_foot.size", std::vector<double>(6, 2));
    declare_parameter("footstep_estimator.right_foot.size", std::vector<double>(6, 2));
    declare_parameter("footstep_estimator.on_ground_threshold", 0.5);
    // Footstep parameters
    auto left_foot_size = this->get_parameter("footstep_estimator.left_foot.size").as_double_array();
    auto right_foot_size = this->get_parameter("footstep_estimator.right_foot.size").as_double_array();

    std_msgs::msg::Int32 stance_foot_msg;
    stance_foot_msg.data = m_current_stance_foot;
    m_stance_foot_publisher->publish(stance_foot_msg);

    initialize_imus();
}

void StateEstimator::lower_imu_callback(sensor_msgs::msg::Imu::SharedPtr msg)
{
    m_imu_estimator.update_imu(*msg, LOWER);
}

void StateEstimator::upper_imu_callback(sensor_msgs::msg::Imu::SharedPtr msg)
{
    m_imu_estimator.update_imu(*msg, UPPER);
}

void StateEstimator::state_callback(sensor_msgs::msg::JointState::SharedPtr msg)
{
    // for(auto& i : msg->position){
    //     if (i == 0.0){
    //         return;
    //     }
    // }
    this->m_joint_estimator.set_joint_states(msg);
    // m_joint_estimator.set_individual_joint_state("right_knee", 0.5);
    m_joint_state_publisher->publish(*msg);
}


void StateEstimator::initialize_imus()
{
    IMU imu_to_set;
    auto imu_base_link = get_parameter("imu_estimator.lower_imu.IMU_exo_base_link").as_string();
    auto imu_position = get_parameter("imu_estimator.lower_imu.IMU_exo_position").as_double_array();
    auto imu_rotation = get_parameter("imu_estimator.lower_imu.IMU_exo_rotation").as_double_array();
    imu_to_set.data.header.frame_id = "lowerIMU";
    imu_to_set.base_frame = imu_base_link;
    imu_to_set.imu_location.translation.x = imu_position[0];
    imu_to_set.imu_location.translation.y = imu_position[1];
    imu_to_set.imu_location.translation.z = imu_position[2];
    tf2::Quaternion tf2_imu_rotation;
    tf2_imu_rotation.setRPY(imu_rotation[0], imu_rotation[1], imu_rotation[2]);
    tf2_imu_rotation.normalize();
    tf2::convert(tf2_imu_rotation, imu_to_set.imu_location.rotation);
    m_imu_estimator.set_imu(imu_to_set, LOWER);
    //
    imu_base_link = get_parameter("imu_estimator.upper_imu.IMU_exo_base_link").as_string();
    imu_position = get_parameter("imu_estimator.upper_imu.IMU_exo_position").as_double_array();
    imu_rotation = get_parameter("imu_estimator.upper_imu.IMU_exo_rotation").as_double_array();
    imu_to_set.data.header.frame_id = "upperIMU";
    imu_to_set.base_frame = imu_base_link;
    imu_to_set.imu_location.translation.x = imu_position[0];
    imu_to_set.imu_location.translation.y = imu_position[1];
    imu_to_set.imu_location.translation.z = imu_position[2];
    tf2_imu_rotation.setRPY(imu_rotation[0], imu_rotation[1], imu_rotation[2]);
    tf2_imu_rotation.normalize();
    tf2::convert(tf2_imu_rotation, imu_to_set.imu_location.rotation);
    m_imu_estimator.set_imu(imu_to_set, UPPER);
}

void StateEstimator::update_foot_frames()
{
    // This script assumes the base foot frames are named LEFT_ORIGIN and RIGHT_ORIGIN;
    // obtain the origin joint
    // IMU& imu = m_imu_estimator.get_imu(LOWER);
    try {
        geometry_msgs::msg::TransformStamped measured_hip_base_angle
            = m_tf_buffer->lookupTransform("lowerIMU", "map", tf2::TimePointZero);
        geometry_msgs::msg::TransformStamped expected_hip_base_angle
            = m_tf_buffer->lookupTransform("hip_base", "right_origin", tf2::TimePointZero);
        //
        tf2::Quaternion tf2_measured_hip_base_angle;
        tf2::Quaternion tf2_expected_hip_base_angle;
        tf2::fromMsg(measured_hip_base_angle.transform.rotation, tf2_measured_hip_base_angle);
        tf2::fromMsg(expected_hip_base_angle.transform.rotation, tf2_expected_hip_base_angle);
        tf2_measured_hip_base_angle.normalize();
        tf2_expected_hip_base_angle.normalize();
        tf2::Quaternion tf2_angle_difference = tf2_measured_hip_base_angle * tf2_expected_hip_base_angle.inverse();
        tf2_angle_difference.normalize();
        geometry_msgs::msg::Quaternion angle_difference;
        tf2::convert(tf2_angle_difference, angle_difference);
        // testing
        tf2::Matrix3x3 m(tf2_measured_hip_base_angle);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        m_joint_estimator.set_individual_joint_state("right_origin", pitch);
        m_joint_estimator.set_individual_joint_state("right_origin_roll", fmod(6.28 - roll, 6.28));
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "error in update_foot_frames: %s", ex.what());
    }
}

void StateEstimator::publish_robot_frames()
{
    // publish IMU frames
    IMU& imu = m_imu_estimator.get_imu(LOWER);
    m_tf_joint_broadcaster->sendTransform(imu.get_imu_rotation());
    update_foot_frames();
    // publish joint frames
    RCLCPP_DEBUG(this->get_logger(), "Number of frames is %i", m_joint_estimator.get_joint_frames().size());
    for (auto i : m_joint_estimator.get_joint_frames()) {
        m_tf_joint_broadcaster->sendTransform(i);
        RCLCPP_DEBUG(this->get_logger(),
            ("\n Set up link " + i.header.frame_id + "\n with child link " + i.child_frame_id).c_str());
    }
    // Publish each joint center of mass
    std::vector<CenterOfMass> joint_com_positions = m_joint_estimator.get_joint_com_positions("map");

    // Update COP
    try {
        geometry_msgs::msg::TransformStamped left_foot_frame
            = m_tf_buffer->lookupTransform("map", "left_origin", tf2::TimePointZero);
        geometry_msgs::msg::TransformStamped right_foot_frame
            = m_tf_buffer->lookupTransform("map", "right_origin", tf2::TimePointZero);

        // get ankle frames to determine where the footsteps are at
        geometry_msgs::msg::TransformStamped left_ankle_frame
            = m_tf_buffer->lookupTransform("map", "left_ankle", tf2::TimePointZero);
        geometry_msgs::msg::TransformStamped right_ankle_frame
            = m_tf_buffer->lookupTransform("map", "right_ankle", tf2::TimePointZero);
    }

    catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Error during publishing of Center of Pressure: %s", ex.what());
    }

    // there was logic to detect stance foot based on COP, but the COP code as been removed
    // instead, we choose a placeholder value 0 for the stance foot (left = -1, right = 1, double = 0)

    // double stance
     m_current_stance_foot = 0;

    std_msgs::msg::Int32 stance_foot_msg;
    stance_foot_msg.data = m_current_stance_foot;
    m_stance_foot_publisher->publish(stance_foot_msg);

    // Update and publish feet height
    march_shared_msgs::msg::FeetHeightStamped feet_height_msg;
    feet_height_msg.header.frame_id = "map";
    feet_height_msg.heights = m_joint_estimator.get_feet_height();
    m_feet_height_publisher->publish(feet_height_msg);

    visualize_joints();
}

geometry_msgs::msg::TransformStamped StateEstimator::get_frame_transform(
    const std::string& target_frame, const std::string& source_frame)
{
    geometry_msgs::msg::TransformStamped frame_transform;
    try {
        frame_transform = m_tf_buffer->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
        return frame_transform;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Source frame: %s", source_frame.c_str());
        RCLCPP_WARN(this->get_logger(), "Target frame: %s", target_frame.c_str());
        RCLCPP_WARN(this->get_logger(), "error in get_frame_transform: %s", ex.what());
        return frame_transform;
    }
}

void StateEstimator::visualize_joints()
{
    // Publish the joint visualizations
    visualization_msgs::msg::Marker joint_markers;
    joint_markers.type = 7;
    joint_markers.header.frame_id = "map";
    joint_markers.id = 0;
    geometry_msgs::msg::Point marker_container;
    tf2::Vector3 joint_endpoint;
    RCLCPP_DEBUG(this->get_logger(), "Published %i markers", joint_markers.points.size());
    joint_markers.action = 0;
    joint_markers.frame_locked = 1;
    joint_markers.scale.x = 0.03;
    joint_markers.scale.y = 0.03;
    joint_markers.scale.z = 0.01;
    joint_markers.pose.position.x = 0.0;
    joint_markers.pose.position.y = 0.0;
    joint_markers.pose.position.z = 0.0;
    joint_markers.pose.orientation.x = 0.0;
    joint_markers.pose.orientation.y = 0.0;
    joint_markers.pose.orientation.z = 0.0;
    joint_markers.pose.orientation.w = 1.0;
    joint_markers.ns = "exo_joint_visualization";
    joint_markers.lifetime.sec = 1;
    joint_markers.color.a = 1.0;
    m_rviz_publisher->publish(joint_markers);
}